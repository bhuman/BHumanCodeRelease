/*
 * @file Logger.cpp
 * The file implements a class that implements an online logger that writes representations
 * in the background while the robot is playing soccer.
 *
 * @author Arne Böckmann
 * @author Thomas Röfer
 */

#include <snappy-c.h>
#include <ctime>
#include "Logger.h"
#include "Tools/Debugging/LogFileFormat.h"
#include "Tools/Debugging/TimingManager.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/InStreams.h"
#ifdef WINDOWS
#define snprintf sprintf_s
#endif

Logger::Logger()
: blackboardVersion(0),
  readIndex(0),
  writeIndex(0),
  frameCounter(0),
  writerIdle(true),
  writerIdleStart(0)
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
    buffer.reserve(parameters.maxBufferSize);
    for(int i = 0; i < parameters.maxBufferSize; ++i)
    {
      buffer.push_back(new MessageQueue());
      buffer.back()->setSize(parameters.blockSize);
    }
    logFilename = generateFilename();
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
            if(getName((MessageID) i) == "id" + representation)
            {
              loggables.push_back(Loggable(&Blackboard::getInstance()[representation.c_str()], (MessageID) i));
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
  char buf[FILENAME_MAX];
  snprintf(buf, sizeof(buf), "%s%d_%s_", parameters.logFilePath.c_str(),
           Global::getSettings().playerNumber, Global::getSettings().robot.c_str());
  time_t now = time(0);
  strftime(buf + strlen(buf), sizeof(buf) - strlen(buf), "%Y-%m-%d_%H-%M-%S", localtime(&now));
  return std::string(buf) + ".log";
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
  for(const Loggable& loggable : loggables)
  {
    out.bin << *loggable.representation;
    if(!out.finishMessage(loggable.id))
      OUTPUT_WARNING("Logging of " << ::getName(loggable.id) << " failed. The buffer is full.");
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
    {
      int diff = (parameters.maxBufferSize + readIndex - writeIndex) % parameters.maxBufferSize;
      OUTPUT_WARNING("Logger buffer is " << diff / (float) parameters.maxBufferSize * 100 << "% free.");
    }
    frameCounter = 0;
  }
}

void Logger::writeThread()
{
  OutBinaryFile file(logFilename);
  ASSERT(file.exists());
  file << logFileCompressed; // Write magic byte that indicates a compressed log file

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
        (unsigned&) compressedBuffer[0] = (unsigned) size;
        file.write(compressedBuffer.data(), size + sizeof(unsigned));
        queue.clear();
      }
      readIndex = (readIndex + 1) % parameters.maxBufferSize;
    }
    else if(!writerIdle)
    {
      writerIdle = true;
      writerIdleStart = SystemCall::getCurrentSystemTime();
    }
}
