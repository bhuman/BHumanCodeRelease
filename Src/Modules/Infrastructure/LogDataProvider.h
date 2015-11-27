/**
 * The file declares a common base class for modules that replay log data.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/MessageQueue/MessageIDs.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Module/Blackboard.h"

class StreamHandler;

class LogDataProvider
{
private:
  ENUM(State,
  {,
    unknown, // No specification available -> replay anyway.
    accept,  // Specification is compatible -> replay.
    convert, // Specification not compatible -> convert.
  });

  State states[numOfDataMessageIDs]; /**< Should the corresponding message ids be replayed? */
  StreamHandler* logStreamHandler = nullptr; /**< The stream specifications of all the types from the log file. */
  StreamHandler* currentStreamHandler = nullptr; /**< The stream specifications of the types in this executable. */

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
