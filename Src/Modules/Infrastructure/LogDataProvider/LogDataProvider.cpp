/**
 * The file implements a common base class for modules that replay log data.
 * @author Thomas Röfer
 */

#include <cstring>
#include "LogDataProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugDataStreamer.h"
#include "Tools/Global.h"
#include "Tools/Streams/Streamable.h"

LogDataProvider::LogDataProvider()
{
  states.fill(unknown);
  if(SystemCall::getMode() == SystemCall::logfileReplay)
    OUTPUT(idTypeInfoRequest, bin, '\0');
}

LogDataProvider::~LogDataProvider()
{
  if(logTypeInfo)
    delete logTypeInfo;
}

bool LogDataProvider::handle(InMessage& message)
{
  if(message.getMessageID() == idTypeInfo)
  {
    if(!logTypeInfo)
      logTypeInfo = new TypeInfo(false);
    message.bin >> *logTypeInfo;
    return true;
  }
  else if(SystemCall::getMode() == SystemCall::logfileReplay && !logTypeInfo)
    return false;
  else if(Blackboard::getInstance().exists(TypeRegistry::getEnumName(message.getMessageID()) + 2)) // +2 to skip the id of the messageID enums.
  {
    if(logTypeInfo)
    {
      if(states[message.getMessageID()] == unknown && strcmp(TypeRegistry::getEnumName(message.getMessageID()), "idLabelImage") != 0)  // <- this is a temporary hack
      {
        // Check whether the current and the logged specifications are the same.
        const char* type = TypeRegistry::getEnumName(message.getMessageID()) + 2;
        states[message.getMessageID()] = currentTypeInfo.areTypesEqual(*logTypeInfo, type, type) ? accept : convert;
        if(states[message.getMessageID()] == convert)
          OUTPUT_WARNING(std::string(type) + " has changed and is converted. Some fields will keep their previous values.");
      }
    }
    if(states[message.getMessageID()] != convert)
      message.bin >> Blackboard::getInstance()[TypeRegistry::getEnumName(message.getMessageID()) + 2];
    else
    {
      ASSERT(logTypeInfo);
      const char* type = TypeRegistry::getEnumName(message.getMessageID()) + 2;

      // Stream into textual representation in memory using type specification of log file.
      OutMapMemory outMap(true, 16384);
      DebugDataStreamer streamer(*logTypeInfo, message.bin, type);
      outMap << streamer;

      // Read from textual representation. Errors are suppressed.
      InMapMemory inMap(outMap.data(), outMap.size(), false);
      inMap >> Blackboard::getInstance()[TypeRegistry::getEnumName(message.getMessageID()) + 2];
    }
    return true;
  }
  else
    return false;
}
