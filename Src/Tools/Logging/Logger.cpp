/*
 * @file Logger.cpp
 * The file implements a class that implements an online logger that writes representations
 * in the background while the robot is playing soccer.
 *
 * @author Arne Böckmann
 * @author Thomas Röfer
 */

#include "Tools/Debugging/Stopwatch.h"
#include "Logger.h"
#include "LogFileFormat.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/AnnotationManager.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/TypeInfo.h"

#ifdef TARGET_ROBOT
#include <sys/stat.h>
#include <sys/types.h>
#include <limits.h>
#endif
#include <snappy-c.h>

Logger::Logger(const std::string& processName, char processIdentifier, int framesPerSecond) :
  processName(processName),
  typeInfo(200000),
  processIdentifier(processIdentifier),
  framesPerSecond(framesPerSecond)
{
  InMapFile stream(std::string("logger") + processName + ".cfg");
  if(stream.exists())
    stream >> parameters;

#ifndef TARGET_ROBOT
  parameters.enabled = false;
  parameters.logFilePath = "Logs/";
#endif

#ifdef TARGET_ROBOT
  if(parameters.enabled && parameters.logToUSB)
  {
    if(SystemCall::usbIsMounted())
    {
      std::string fullPath = parameters.logUSBFilePath + Global::getSettings().headName;
      _mkdir(fullPath.c_str());
      parameters.logFilePath = fullPath + "/";
    }
  }

#endif

  if(parameters.enabled)
  {
    typeInfo << TypeInfo();
    InMapFile stream2("teamList.cfg");
    if(stream2.exists())
      stream2 >> teamList;

    buffer.reserve(parameters.maxBufferSize);
    for(int i = 0; i < parameters.maxBufferSize; ++i)
    {
      buffer.push_back(new MessageQueue());
      buffer.back()->setSize(parameters.blockSize);
    }
    writerThread.setPriority(parameters.writePriority);
  }
}

Logger::~Logger()
{
  if(frameCounter)
    framesToWrite.post();
  writerThread.stop();
  for(MessageQueue* m : buffer)
    delete m;
}

void Logger::execute()
{
  if(parameters.enabled)
  {
    if(Blackboard::getInstance().getVersion() != blackboardVersion)
    {
      loggables.clear();
      for(const std::string& representation : parameters.representations)
        if(Blackboard::getInstance().exists(representation.c_str()))
        {
          FOREACH_ENUM(MessageID, id, numOfDataMessageIDs)
            if(TypeRegistry::getEnumName(id) == "id" + representation)
            {
              loggables.push_back(Loggable(&Blackboard::getInstance()[representation.c_str()], id));
              goto found;
            }
          OUTPUT_WARNING(processName << "Logger: " << representation << " has no message id.");
          FAIL(processName << "Logger: " << representation << " has no message id.");
        found:
          ;
        }
        else
        {
          OUTPUT_WARNING(processName << "Logger: " << representation << " is not available in blackboard.");
          FAIL(processName << "Logger: " << representation << " is not available in blackboard.");
        }

      blackboardVersion = Blackboard::getInstance().getVersion();
    }

    beginFrame(Time::getCurrentSystemTime());
    Cabsl<Logger>::execute(OptionInfos::getOption("Root"));
    endFrame();
  }
}

std::string Logger::generateFilename() const
{
  static_assert(Settings::highestValidPlayerNumber < 10, "This code does not work with player numbers >= 10.");
  if(receivedGameControllerPacket)
  {
    const OpponentTeamInfo& opponentTeamInfo = static_cast<const OpponentTeamInfo&>(Blackboard::getInstance()["OpponentTeamInfo"]);
    for(const auto& team : teamList.teams)
      if(team.number == opponentTeamInfo.teamNumber)
      {
        std::string filename = team.name + "_";
        const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
        filename += gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT ? "ShootOut_" :
                    gameInfo.firstHalf ? "1stHalf_" : "2ndHalf_";
        filename += static_cast<char>(Global::getSettings().playerNumber + '0');
        return parameters.logFilePath + processName + "_" + Global::getSettings().headName + "_" + Global::getSettings().bodyName + "_" + Global::getSettings().scenario + "_" + Global::getSettings().location + "__" + filename;
      }
  }
  return parameters.logFilePath + processName + "_" + Global::getSettings().headName + "_" + Global::getSettings().bodyName + "_" + Global::getSettings().scenario + "_" + Global::getSettings().location + "__" + "Testing_" + static_cast<char>(Global::getSettings().playerNumber + '0');
}

void Logger::logFrame()
{
  if(writeIndex == (readIndex + parameters.maxBufferSize - 1) % parameters.maxBufferSize)
  {
    // Buffer is full, can't do anything this frame
    OUTPUT_WARNING(processName << "Logger: Writer thread too slow, discarding frame.");
    return;
  }

  OutMessage& out = buffer[writeIndex]->out;

  out.bin << processIdentifier;
  out.finishMessage(idProcessBegin);

  // Stream all representations to the queue
  STOPWATCH("Logger")
  {
    for(const Loggable& loggable : loggables)
    {
      out.bin << *loggable.representation;
      if(!out.finishMessage(loggable.id))
        OUTPUT_WARNING("Logging of " << TypeRegistry::getEnumName(loggable.id) << " failed. The buffer is full.");
    }

    // Append annotations
    Global::getAnnotationManager().getOut().copyAllMessages(*buffer[writeIndex]);
  }

  // Append timing data if any
  MessageQueue& timingData = Global::getTimingManager().getData();
  if(timingData.getNumberOfMessages() > 0)
    timingData.copyAllMessages(*buffer[writeIndex]);
  else
    OUTPUT_WARNING(processName << "Logger: No timing data available.");

  out.bin << processIdentifier;
  out.finishMessage(idProcessFinished);

  // One block is used per second.
  if(++frameCounter == framesPerSecond)
  {
    // The next call to logFrame will use a new block.
    writeIndex = (writeIndex + 1) % parameters.maxBufferSize;
    framesToWrite.post(); // Signal to the writer thread that another block is ready
    if(parameters.debugStatistics)
      OUTPUT_WARNING(processName << "Logger buffer is "
                     << ((parameters.maxBufferSize + readIndex - writeIndex) % parameters.maxBufferSize) /
                     static_cast<float>(parameters.maxBufferSize) * 100.f
                     << "% free.");
    frameCounter = 0;
  }
}

#ifdef TARGET_ROBOT
void Logger::_mkdir(const char* dir)
{
  /* Copied from https://gist.github.com/JonathonReinhart/8c0d90191c38af2dcadb102c4e202950 */
  const size_t len = strlen(dir);
  char _path[PATH_MAX];
  char* p;

  errno = 0;

  /* Copy string so its mutable */
  if(len > sizeof(_path) - 1)
  {
    errno = ENAMETOOLONG;
    return;
  }
  strcpy(_path, dir);

  /* Iterate the string */
  for(p = _path + 1; *p; p++)
  {
    if(*p == '/')
    {
      /* Temporarily truncate */
      *p = '\0';

      if(mkdir(_path, S_IRWXU) != 0)
      {
        if(errno != EEXIST)
          return;
      }

      *p = '/';
    }
  }

  if(mkdir(_path, S_IRWXU) != 0)
  {
    if(errno != EEXIST)
      return;
  }

  return;
}

#endif

void Logger::writeThread()
{
  Thread::nameThread(std::string("Logger") + processName);
  BH_TRACE_INIT(processName.c_str());
  const size_t compressedSize = snappy_max_compressed_length(parameters.blockSize + 2 * sizeof(unsigned));
  std::vector<char> compressedBuffer(compressedSize + sizeof(unsigned)); // Also reserve 4 bytes for header
  OutBinaryFile* file = nullptr;

  while(writerThread.isRunning()) // Check if we are expecting more data
    if(framesToWrite.wait(100)) // Wait 100 ms for new data then check again if we should quit
    {
      writerIdle = false;
      MessageQueue& queue = *buffer[readIndex];
      if(queue.getNumberOfMessages() > 0)
      {
        size_t size = compressedSize;
        VERIFY(snappy_compress(queue.getStreamedData(), queue.getStreamedSize(),
                               compressedBuffer.data() + sizeof(unsigned), &size) == SNAPPY_OK);
        reinterpret_cast<unsigned&>(compressedBuffer[0]) = static_cast<unsigned>(size);
        if(!file)
        {
          // find next free log filename
          std::string num = "";
          for(int i = 0; i < 100; ++i)
          {
            if(i)
            {
              char buf[6];
              sprintf(buf, "_(%02d)", i);
              num = buf;
            }
            InBinaryFile stream(logFilename + num + ".log");
            if(!stream.exists())
              break;
          }
          logFilename += num + ".log";

          file = new OutBinaryFile(logFilename);
          ASSERT(file->exists());
          *file << Logging::logFileMessageIDs;
          queue.writeMessageIDs(*file);
          *file << Logging::logFileTypeInfo;
          file->write(typeInfo.data(), typeInfo.size());
          *file << Logging::logFileCompressed; // Write magic byte that indicates a compressed log file
        }
        if(SystemCall::getFreeDiskSpace(logFilename.c_str())
           < (static_cast<unsigned long long>(parameters.minFreeSpace) << 20) + size + sizeof(unsigned))
          break;
        file->write(compressedBuffer.data(), size + sizeof(unsigned));
        queue.clear();
      }
      readIndex = (readIndex + 1) % parameters.maxBufferSize;
    }
    else if(!writerIdle)
    {
      writerIdle = true;
      writerIdleStart = Time::getCurrentSystemTime();
    }

  if(file)
    delete file;
}
