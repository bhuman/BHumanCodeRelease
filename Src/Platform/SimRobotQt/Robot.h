/**
* @file Platform/SimRobotQt/Robot.h
*
* This file declares the class Robot.
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/ProcessFramework/ProcessFramework.h"

class RobotConsole;

/**
* The class implements a robot as a list of processes.
*/
class Robot : public ProcessList
{
private:
  RobotConsole* robotProcess; /**< A pointer to the process that simulates the physical robots. */
  std::string name; /**< The name of the robot. */

public:
  /**
  * Constructor.
  * @param name The name of the robot.
  */
  Robot(const char* name);

  /**
  * The function updates all sensors and sends motor commands to SimRobot.
  */
  void update();

  /**
  * The function returns the name of the robot.
  * @return The name of the robot.
  */
  const char* getName() const {return name.c_str();}

  /**
  * The function returns a pointer to the process that simulates the physical robots.
  * @return The pointer to the process.
  */
  RobotConsole* getRobotProcess() const {return robotProcess;}

  /**
  * The function connects a sender and a receiver.
  * @param sender The sender.
  * @param receiver The receiver.
  */
  static void connect(SenderList* sender, ReceiverList* receiver);

private:
  /**
  * The function looks up a sender.
  * @param senderName The name of the sender. If the process name is missing
  *                   i.e. senderName starts with a dot, the first process with a
  *                   sender that matches the rest of the name is used.
  * @return A pointer to the sender or 0 if no sender exists with the specified name.
  */
  SenderList* getSender(const std::string& senderName);

  /**
  * The function looks up a receiver.
  * @param receiverName The name of the receiver.
  * @return A pointer to the receiver or 0 if no receiver exists with the specified name.
  */
  ReceiverList* getReceiver(const std::string& receiverName);
};
