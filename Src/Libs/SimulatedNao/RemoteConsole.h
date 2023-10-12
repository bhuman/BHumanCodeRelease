/**
 * @file SimulatedNao/RemoteConsole.h
 * Declaration of a class representing a thread that communicates with a remote robot via TCP.
 * @author Thomas Röfer
 */

#pragma once

#include "Debugging/TcpConnection.h"
#include "RobotConsole.h"

class ConsoleRoboCupCtrl;

/**
 * @class RemoteConsole
 *
 * A class representing a thread that communicates with a remote robot via TCP.
 */
class RemoteConsole : public RobotConsole, public TcpConnection
{
private:
  const std::string ip; /**< The ip of the robot. */
  int bytesTransferred = 0; /**< The number of bytes transferred so far. */
  float transferSpeed = 0.f; /**< The transfer speed in kb/s. */
  unsigned timestamp = 0; /**< The time when the transfer speed was measured. */
  char messageIdOffset = 0; /**< The offset to add to ids of the robot to convert them to local ids. */
  bool numOfDataMessageIDsRequestSent = false; /**< Was the request to retrieve the \c numOfDataMessageIDs from the remote robot sent? */
  bool numOfDataMessageIDsReceived = false; /**< Was \c numOfDataMessageIDs already received from remote robot? */

public:
  /**
   * @param robotName The name of the robot.
   * @param ip The ip address of the robot.
   * @param ctrl A pointer to the controller object.
   */
  RemoteConsole(const std::string& robotName, const std::string& ip, ConsoleRoboCupCtrl* ctrl);

  /**
   * The function is called to announce the termination of the thread.
   */
  void announceStop() override;

  /**
   * The function must be called to exchange data with SimRobot.
   * It sends the motor commands to SimRobot and acquires new sensor data.
   */
  void update() override;

  /**
   * The function returns the name of the robot.
   *
   * @return The name.
   */
  const std::string getName() const override { return robotName; }

protected:
  /**
   * That function is called once before the first main(). It can be used
   * for things that can't be done in the constructor.
   */
  void init() override
  {
    RobotConsole::init();
    Thread::nameCurrentThread(robotName + ".RemoteConsole");
  }

  /**
   * The function is called from the framework once in every frame.
   */
  bool main() override;

  /**
   * The function is called for every incoming debug message.
   * @param message An interface to read the message from the queue.
   * @return Has the message been handled?
   */
  bool handleMessage(MessageQueue::Message message) override;

private:

  /**
   * The function connects to another thread.
   */
  void connect();

  /**
   * Changes message ids in a buffer by conditionally adding an offset.
   * @param buffer The start of the buffer.
   * @param size The size of the buffer.
   * @param offset The offset that is applied.
   */
  void correctMessageIDs(unsigned char* buffer, size_t size, char offset);
};
