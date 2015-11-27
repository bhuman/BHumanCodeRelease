/**
 * The file implements a common base class for modules that replay log data.
 * @author Thomas Röfer
 */

#include "LogDataProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDataStreamer.h"
#include "Tools/Global.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/StreamHandler.h"
#include <cstring>

LogDataProvider::LogDataProvider()
{
  memset(states, unknown, sizeof(states));
}

LogDataProvider::~LogDataProvider()
{
  if(logStreamHandler)
    delete logStreamHandler;
  if(currentStreamHandler)
    delete currentStreamHandler;
}

bool LogDataProvider::handle(InMessage& message)
{
  if(message.getMessageID() == idStreamSpecification)
  {
    if(!logStreamHandler)
      logStreamHandler = new StreamHandler;
    message.bin >> *logStreamHandler;
    return true;
  }
  else if(Blackboard::getInstance().exists(::getName(message.getMessageID()) + 2))
  {
    if(logStreamHandler)
    {
      if(states[message.getMessageID()] == unknown)
      {
        // Stream representation from blackboard to acquire the current specification
        OutBinarySize dummy;
        dummy << Blackboard::getInstance()[::getName(message.getMessageID()) + 2];

        if(!currentStreamHandler)
          currentStreamHandler = new StreamHandler;

        // Make a copy of current specifications through streaming.
        // As a result, all type names in currentStreamHandler are demangled.
        OutBinarySize size;
        size << Global::getStreamHandler();

        char* buffer = new char[size.getSize()];
        OutBinaryMemory out(buffer);
        out << Global::getStreamHandler();
        InBinaryMemory in(buffer);
        in >> *currentStreamHandler;

        // Check whether the current and the logged specifications are the same.
        const char* type = ::getName(message.getMessageID()) + 2;
        states[message.getMessageID()] = currentStreamHandler->areSpecificationsForTypesCompatible(*logStreamHandler, type, type) ? accept : convert;
        if(states[message.getMessageID()] == convert)
          OUTPUT_WARNING(std::string(type) + " has changed and is converted. Some fields will keep their previous values.");
      }
    }
    if(states[message.getMessageID()] != convert)
      message.bin >> Blackboard::getInstance()[::getName(message.getMessageID()) + 2];
    else
    {
      ASSERT(logStreamHandler);
      const char* type = ::getName(message.getMessageID()) + 2;

      // Stream into textual representation in memory using type specification of log file.
      // Determine required buffer size first.
      OutMapSize outMapSize(true);
      {
        DebugDataStreamer streamer(*logStreamHandler, message.bin, type);
        outMapSize << streamer;
        message.resetReadPosition();
      }
      char* mapBuffer = new char[outMapSize.getSize()];
      {
        OutMapMemory outMap(mapBuffer, true);
        DebugDataStreamer streamer(*logStreamHandler, message.bin, type);
        outMap << streamer;
      }

      // Read from textual representation. Errors are suppressed.
      InMapMemory inMap(mapBuffer, outMapSize.getSize(), false);
      inMap >> Blackboard::getInstance()[::getName(message.getMessageID()) + 2];

      // Clean up
      delete [] mapBuffer;
    }
    return true;
  }
  else
    return false;
}
