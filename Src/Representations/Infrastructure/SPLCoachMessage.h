/**
 * @file SPLCoachMessage.h
 * The file declares a class that encapsulates the structure SPLCoachMessage
 * defined in the file SPLCoachMessage.h that is provided with the GameController.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"

class SPLCoachMessage : public RoboCup::SPLCoachMessage, public Streamable
{
private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out);

  using RoboCup::SPLCoachMessage::header; // Hide, because it is not streamed
  using RoboCup::SPLCoachMessage::version; // Hide, because it is not streamed
  using RoboCup::SPLCoachMessage::message; // Use setter and getter instead

public:
  /** Default constructor. */
  SPLCoachMessage();

  /**
   * Create complete coach message.
   * @param team The number of the team.
   * @param message The message.
   */
  SPLCoachMessage(uint8_t team, const std::string& message);

  /**
   * Sets the message and handles the size limit.
   * @param message The new message.
   */
  void setMessage(const std::string& message);

  /**
   * Gets the message and handles the size limit.
   * @return The current message.
   */
  std::string getMessage() const;
};
