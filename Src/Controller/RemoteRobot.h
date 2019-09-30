/**
 * @file Controller/RemoteRobot.h
 * Declaration of a class representing a thread that communicates with a remote robot via TCP.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Debugging/TcpConnection.h"
#include "RobotConsole.h"
#include "SimulatedRobot.h"

/**
 * @class RemoteRobot
 *
 * A class representing a thread that communicates with a remote robot via TCP.
 */
class RemoteRobot : public RobotConsole, public TcpConnection
{
private:
  const std::string name; /**< The name of the robot. */
  const std::string ip; /**< The ip of the robot. */
  int bytesTransfered = 0; /**< The number of bytes transfered so far. */
  float transferSpeed = 0.f; /**< The transfer speed in kb/s. */
  unsigned timestamp = 0; /**< The time when the transfer speed was measured. */
  SimulatedRobot simulatedRobot; /**< The interface to simulated objects. */
  SimRobotCore2::Body* puppet; /**< A pointer to the puppet when there is one. Otherwise 0. */

public:
  /**
   * @param name The name of the robot.
   * @param ip The ip address of the robot.
   */
  RemoteRobot(const std::string& name, const std::string& ip);

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
  const std::string getName() const override { return name; }

protected:
  /**
   * That function is called once before the first main(). It can be used
   * for things that can't be done in the constructor.
   */
  void init() override
  {
    RobotConsole::init();
    Thread::nameCurrentThread(name + ".RemoteRobot");
  }

  /**
   * The function is called from the framework once in every frame.
   */
  bool main() override;

private:

  /**
   * The function connects to another thread.
   */
  void connect();
};
