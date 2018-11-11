/**
 * @file Controller/RemoteRobot.h
 * Declaration of a class representing a process that communicates with a remote robot via TCP.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Debugging/TcpConnection.h"
#include "RobotConsole.h"
#include "SimulatedRobot.h"

/**
 * @class RemoteRobot
 * A class representing a process that communicates with a remote robot via TCP.
 */
class RemoteRobot : public RobotConsole, public TcpConnection, private Thread
{
private:
  Receiver<MessageQueue> theDebugReceiver;
  DebugSender<MessageQueue> theDebugSender;
  const std::string name; /**< The name of the robot. */
  const std::string ip; /**< The ip of the robot. */
  int bytesTransfered = 0; /**< The number of bytes transfered so far. */
  float transferSpeed = 0.f; /**< The transfer speed in kb/s. */
  unsigned timeStamp = 0; /**< The time when the transfer speed was measured. */
  SimulatedRobot simulatedRobot; /**< The interface to simulated objects. */
  SimRobotCore2::Body* puppet; /**< A pointer to the puppet when there is one. Otherwise 0. */

public:
  /**
   * @param name The name of the robot.
   * @param ip The ip address of the robot.
   */
  RemoteRobot(const std::string& name, const std::string& ip);

  ~RemoteRobot() { Thread::stop(); setGlobals(); }

  /**
   * The function starts the process.
   */
  void start() { Thread::start(this, &RemoteRobot::run); }

  /**
   * The function is called to announce the termination of the process.
   */
  void announceStop() override;

  /**
   * The function must be called to exchange data with SimRobot.
   * It sends the motor commands to SimRobot and acquires new sensor data.
   */
  void update() override;

  /**
   * The function returns the name of the robot.
   * @return The name.
   */
  const std::string& getName() const { return name; }

private:
  /**
   * The main loop of the process.
   */
  void run();

  /**
   * The function connects to another process.
   */
  void connect();

  /**
   * The function is called from the framework once in every frame.
   */
  bool main() override;
};
