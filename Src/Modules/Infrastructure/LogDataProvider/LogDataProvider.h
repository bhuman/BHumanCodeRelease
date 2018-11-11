/**
 * The file declares a common base class for modules that replay log data.
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/TypeInfo.h"

// No verify when replaying logfiles
#ifndef NDEBUG
#undef _MODULE_VERIFY
#define _MODULE_VERIFY(r) \
  if(SystemCall::getMode() != SystemCall::logfileReplay) \
    ModuleBase::verify(&r);
#endif

class LogDataProvider
{
private:
  ENUM(State,
  {,
    unknown, // No specification available -> replay anyway.
    accept,  // Specification is compatible -> replay.
    convert, // Specification not compatible -> convert.
  });

  std::array<State, numOfDataMessageIDs> states; /**< Should the corresponding message ids be replayed? */
  TypeInfo* logTypeInfo = nullptr; /**< The specifications of all the types from the log file. */
  const TypeInfo currentTypeInfo; /**< The specifications of the types in this executable. */

public:
  LogDataProvider();
  ~LogDataProvider();

protected:
  /**
   * Handle message by writing the data contained into the corresponding blackboard entry.
   * @param message The message to be handled.
   * @return Was the message handled?
   */
  bool handle(InMessage& message);
};
