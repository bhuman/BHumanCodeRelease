/*
 * @file Logger.cpp
 * The file implements a class that implements an online logger that writes representations
 * in the background while the robot is playing soccer.
 *
 * @author Arne Böckmann
 * @author Thomas Röfer
 */

#include <snappy-c.h>
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/AnnotationManager.h"
#include "Tools/MessageQueue/LogFileFormat.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/StreamHandler.h"
#include "Logger.h"
#ifdef WINDOWS
#define snprintf sprintf_s
#endif

Logger::Logger()
{
  InMapFile stream("logger.cfg");
  if(stream.exists())
    stream >> parameters;

#ifndef TARGET_ROBOT
  parameters.enabled = false;
  parameters.logFilePath = "Logs/";
#endif

  if(parameters.enabled)
  {
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
          int i;
          for(i = 0; i < numOfDataMessageIDs; ++i)
            if(getName(static_cast<MessageID>(i)) == "id" + representation)
            {
              loggables.push_back(Loggable(&Blackboard::getInstance()[representation.c_str()], static_cast<MessageID>(i)));
              break;
            }
          if(i == numOfDataMessageIDs)
            OUTPUT_WARNING("Logger: " << representation << " has no message id.");
        }
        else
        { // This pair of braces avoids strange bug in VS2013
          OUTPUT_WARNING("Logger: " << representation << " is not available in blackboard.");
        }

      blackboardVersion = Blackboard::getInstance().getVersion();
    }

    beginFrame(SystemCall::getCurrentSystemTime());
    Cabsl<Logger>::execute(OptionInfos::getOption("Root"));
    endFrame();
  }
}

std::string Logger::generateFilename() const
{
  if(receivedGameControllerPacket)
  {
    const OpponentTeamInfo& opponentTeamInfo = static_cast<const OpponentTeamInfo&>(Blackboard::getInstance()["OpponentTeamInfo"]);
    for(const auto& team : teamList.teams)
      if(team.number == opponentTeamInfo.teamNumber)
      {
        std::string filename = team.name + "_";
        const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
        filename += gameInfo.secondaryState == STATE2_PENALTYSHOOT ? "ShootOut_" :
                    gameInfo.firstHalf ? "1stHalf_" : "2ndHalf_";
        filename += static_cast<char>(Global::getSettings().playerNumber + '0');
        return parameters.logFilePath + filename;
      }
  }
  return parameters.logFilePath + "Testing";
}

void Logger::createStreamSpecification()
{
  OutBinarySize size;
  size << Global::getStreamHandler();
  streamSpecification.resize(size.getSize());
  OutBinaryMemory stream(streamSpecification.data());
  stream << Global::getStreamHandler();
}

void Logger::logFrame()
{
  if(writeIndex == (readIndex + parameters.maxBufferSize - 1) % parameters.maxBufferSize)
  { // Buffer is full, can't do anything this frame
    OUTPUT_WARNING("Logger: Writer thread too slow, discarding frame.");
    return;
  }

  OutMessage& out = buffer[writeIndex]->out;

  out.bin << 'c';
  out.finishMessage(idProcessBegin);

  // Stream all representations to the queue
  STOPWATCH("Logger")
  {
    for(const Loggable& loggable : loggables)
    {
      out.bin << *loggable.representation;
      if(!out.finishMessage(loggable.id))
        OUTPUT_WARNING("Logging of " << ::getName(loggable.id) << " failed. The buffer is full.");
    }

    // Append annotations
    Global::getAnnotationManager().getOut().copyAllMessages(*buffer[writeIndex]);
  }

  // Append timing data if any
  MessageQueue& timingData = Global::getTimingManager().getData();
  if(timingData.getNumberOfMessages() > 0)
    timingData.copyAllMessages(*buffer[writeIndex]);
  else
    OUTPUT_WARNING("Logger: No timing data available.");

  out.bin << 'c';
  out.finishMessage(idProcessFinished);

  // Cognition runs at 60 fps. Therefore use a new block every 60 frames.
  // Thus one block is used per second.
  if(++frameCounter == 60)
  {
    // The next call to logFrame will use a new block.
    writeIndex = (writeIndex + 1) % parameters.maxBufferSize;
    framesToWrite.post(); // Signal to the writer thread that another block is ready
    if(parameters.debugStatistics)
      OUTPUT_WARNING("Logger buffer is "
                     << ((parameters.maxBufferSize + readIndex - writeIndex) % parameters.maxBufferSize) /
                        static_cast<float>(parameters.maxBufferSize) * 100.f
                     << "% free.");
    frameCounter = 0;
  }
}

void Logger::writeThread()
{
  const size_t compressedSize = snappy_max_compressed_length(parameters.blockSize + 2 * sizeof(unsigned));
  std::vector<char> compressedBuffer(compressedSize + sizeof(unsigned)); // Also reserve 4 bytes for header

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
        (unsigned&)compressedBuffer[0] = static_cast<unsigned>(size);
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
          *file << logFileMessageIDs;
          queue.writeMessageIDs(*file);
          *file << logFileStreamSpecification;
          file->write(streamSpecification.data(), streamSpecification.size());
          *file << logFileCompressed; // Write magic byte that indicates a compressed log file
        }
        file->write(compressedBuffer.data(), size + sizeof(unsigned));
        queue.clear();
      }
      readIndex = (readIndex + 1) % parameters.maxBufferSize;
    }
    else if(!writerIdle)
    {
      writerIdle = true;
      writerIdleStart = SystemCall::getCurrentSystemTime();
    }

  if(file)
    delete file;
}
