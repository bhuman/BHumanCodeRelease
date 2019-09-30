/**
 * @file Logger.cpp
 *
 * This file implements a class that writes a subset of representations into
 * log files. The representations can stem from multiple parallel threads.
 * The class maintains a buffer of message queues that can be claimed by
 * individual threads, filled with data, and given back to the logger for
 * writing them to the log file.
 *
 * @author Thomas Röfer
 */

#include "Logger.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/Debugging/AnnotationManager.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/TimingManager.h"
#include "Tools/Global.h"
#include "Tools/Logging/LoggingTools.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Settings.h"
#include "Tools/Streams/TypeInfo.h"
#include <cstring>
#include <snappy-c.h>

#undef PRINT
#ifndef TARGET_ROBOT
#define PRINT(message) OUTPUT_WARNING(message)
#elif defined NDEBUG
#include <cstdlib>
#define PRINT(message) \
  do \
  { \
    OUTPUT_ERROR(message); \
    abort(); \
  } \
  while(false)
#else
#define PRINT(message) FAIL(message)
#endif

Logger::Logger(const Configuration& config)
  : typeInfo(200000)
{
  std::memset(gameInfoThreadName, 0, sizeof(gameInfoThreadName));
  InMapFile stream("logger.cfg");
  ASSERT(stream.exists());
  stream >> *this;
  if(!path.empty() && path.back() != '/')
    path += "/";

  // Report wrong logger configuration (even in the simulator).
  // This check is only executed for the initial configuration, because on the robot that is usually the one that is used.
  if(enabled)
    for(const auto& rpt : representationsPerThread)
    {
      for(const auto& thread : config.threads)
        if(thread.name == rpt.thread)
        {
          for(const std::string& loggerRepresentation : rpt.representations)
          {
            for(const std::string& defaultRepresentation : config.defaultRepresentations)
              if(loggerRepresentation == defaultRepresentation)
              {
                PRINT("Logger: Thread " << rpt.thread << " should not log default representation " << defaultRepresentation);
                goto representationFound;
              }

            for(const auto& representationProvider : thread.representationProviders)
              if(loggerRepresentation == representationProvider.representation)
                goto representationFound;
            PRINT("Logger: Thread " << rpt.thread << " does not contain representation " << loggerRepresentation);
          representationFound:;
          }
          goto threadFound;
        }
      PRINT("Logger: Thread " << rpt.thread << " not found");
    threadFound:;
    }

#ifndef TARGET_ROBOT
  enabled = false;
  path = "Logs/";
#endif

  if(enabled)
  {
    typeInfo << TypeInfo();
    InMapFile stream("teamList.cfg");
    if(stream.exists())
      stream >> teamList;

    buffers.resize(numOfBuffers);
    for(MessageQueue& buffer : buffers)
    {
      buffer.setSize(sizeOfBuffer);
      buffersAvailable.push(&buffer);
    }

    writerThread.setPriority(writePriority);
    writerThread.start(this, &Logger::writer);
  }
}

void Logger::execute(const std::string& threadName)
{
  if(!enabled)
    return;

  if((!*gameInfoThreadName || threadName == gameInfoThreadName)
     && Blackboard::getInstance().exists("GameInfo") && Blackboard::getInstance().exists("OpponentTeamInfo"))
  {
    const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    const bool loggingNow = gameInfo.state != STATE_INITIAL && gameInfo.state != STATE_FINISHED;
    if(loggingNow && !*gameInfoThreadName)
    {
      std::string description = "Testing";
      if(gameInfo.packetNumber || gameInfo.secsRemaining != 0) // Packet from GameController
      {
        const OpponentTeamInfo& opponentTeamInfo = static_cast<const OpponentTeamInfo&>(Blackboard::getInstance()["OpponentTeamInfo"]);
        for(const auto& team : teamList.teams)
          if(team.number == opponentTeamInfo.teamNumber)
          {
            description = team.name + "_"
                          + (gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT ? "ShootOut"
                             : gameInfo.firstHalf ? "1stHalf" : "2ndHalf");
            break;
          }
      }

      SYNC;
      filename = path + LoggingTools::createName("", Global::getSettings().headName, Global::getSettings().bodyName,
                                                 Global::getSettings().scenario, Global::getSettings().location,
                                                 description, Global::getSettings().playerNumber);
      std::strncpy(gameInfoThreadName, threadName.c_str(), sizeof(gameInfoThreadName) - 1);
    }
    logging = loggingNow;

    if(!logging && hasLogged && buffersToWrite.empty())
    {
      SystemCall::say("Log file written");
      hasLogged = false;
    }
  }

  if(logging)
  {
    for(const RepresentationsPerThread& rpt : representationsPerThread)
      if(rpt.thread == threadName && !rpt.representations.empty())
      {
        MessageQueue* buffer = nullptr;
        {
          SYNC;
          if(!buffersAvailable.empty())
          {
            buffer = buffersAvailable.top();
            buffersAvailable.pop();
          }
        }
        if(!buffer)
        {
          OUTPUT_WARNING("Logger: No buffer available!");
          return;
        }

        buffer->out.bin << threadName;
        buffer->out.finishMessage(idFrameBegin);

        for(const std::string& representation : rpt.representations)
#ifndef NDEBUG
          if(Blackboard::getInstance().exists(representation.c_str()))
#endif
          {
            buffer->out.bin << Blackboard::getInstance()[representation.c_str()];
            if(!buffer->out.finishMessage(static_cast<MessageID>(TypeRegistry::getEnumValue(typeid(MessageID).name(), "id" + representation))))
              OUTPUT_WARNING("Logger: Representation " << representation << " did not fit into buffer!");
          }
#ifndef NDEBUG
          else
            OUTPUT_WARNING("Logger: Representation " << representation << " does not exists!");
#endif

        Global::getAnnotationManager().getOut().copyAllMessages(*buffer);
        Global::getTimingManager().getData().copyAllMessages(*buffer);
        buffer->out.bin << threadName;
        buffer->out.finishMessage(idFrameFinished);
        {
          SYNC;
          buffersToWrite.push_back(buffer);
        }
        framesToWrite.post();
        hasLogged = true;
        break;
      }
  }
}

Logger::~Logger()
{
  writerThread.announceStop();
  framesToWrite.post();
  writerThread.stop();
}

void Logger::writer()
{
  Thread::nameCurrentThread("Logger");
  BH_TRACE_INIT("Logger");

  const size_t compressedSize = snappy_max_compressed_length(sizeOfBuffer + 2 * sizeof(unsigned));
  std::vector<char> compressedBuffer(compressedSize + sizeof(unsigned)); // Also reserve 4 bytes for header
  OutBinaryFile* file = nullptr;
  std::string completeFilename;

  while(true)
  {
    framesToWrite.wait();
    if(!writerThread.isRunning()
       || (!completeFilename.empty()
           && SystemCall::getFreeDiskSpace(completeFilename.c_str()) < static_cast<unsigned long long>(minFreeDriveSpace) << 20))
      break;

    // This assumes that reading the front is threadsafe.
    MessageQueue* buffer = buffersToWrite.front();

    if(!file)
    {
      // find next free log filename
      for(int i = 0; i < 100; ++i)
      {
        completeFilename = filename + (i ? "_(" + ((i < 10 ? "0" : "") + std::to_string(i)) + ")" : "") + ".log";
        InBinaryFile stream(completeFilename);
        if(!stream.exists())
          break;
      }

      file = new OutBinaryFile(completeFilename);
      if(!file->exists())
      {
        OUTPUT_WARNING("Logger: File " << completeFilename << " could not be created!");
        break;
      }

      *file << LoggingTools::logFileMessageIDs;
      buffer->writeMessageIDs(*file);
      *file << LoggingTools::logFileTypeInfo;
      file->write(typeInfo.data(), typeInfo.size());
      *file << LoggingTools::logFileCompressed;
    }

    size_t size = compressedSize;
    VERIFY(snappy_compress(buffer->getStreamedData(), buffer->getStreamedSize(),
                           compressedBuffer.data() + sizeof(unsigned), &size) == SNAPPY_OK);
    reinterpret_cast<unsigned&>(compressedBuffer[0]) = static_cast<unsigned>(size);
    buffer->clear();

    {
      SYNC;
      buffersToWrite.pop_front();
      buffersAvailable.push(buffer);
    }

    file->write(compressedBuffer.data(), size + sizeof(unsigned));
  }

  if(file)
    delete file;
}
