/**
 * @file SPLCoachMessage.cpp
 * The file implements a class that encapsulates the structure SPLCoachMessage
 * defined in the file SPLCoachMessage.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "SPLCoachMessage.h"
#include <cstring>

SPLCoachMessage::SPLCoachMessage()
{
  team = message[0] = 0;
}

SPLCoachMessage::SPLCoachMessage(uint8_t team, const std::string& message)
{
  this->team = team;
  setMessage(message);
}

void SPLCoachMessage::serialize(In* in, Out* out)
{
  std::string message = getMessage();

  STREAM_REGISTER_BEGIN;
  STREAM(team); // unique team number
  STREAM(message); // The message up to 40 characters
  STREAM_REGISTER_FINISH;

  if(in)
    setMessage(message);
}

void SPLCoachMessage::setMessage(const std::string& message)
{
  if(message.empty())
    this->message[0] = 0;
  else
    strncpy((char*) this->message, &message[0], sizeof(this->message));
}

std::string SPLCoachMessage::getMessage() const
{
  char message[sizeof(this->message) + 1];
  strncpy(message, (char*) this->message, sizeof(this->message));
  message[sizeof(this->message)] = 0;
  return message;
}
