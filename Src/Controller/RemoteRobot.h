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
class RemoteRobot : public RobotConsole, public TcpConnection, private Thread<RemoteRobot>
{
private:
  DEBUGGING;

  char name[80]; /**< The name of the robot. */
  char ip[80]; /**< The ip of the robot. */
  int bytesTransfered; /**< The number of bytes transfered so far. */
  float transferSpeed; /**< The transfer speed in kb/s. */
  unsigned timeStamp; /**< The time when the transfer speed was measured. */
  SimulatedRobot simulatedRobot; /**< The interface to simulated objects. */
  SimRobotCore2::Body* puppet; /**< A pointer to the puppet when there is one. Otherwise 0. */

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
  virtual bool main();

public:
  /**
  * Constructor.
  * @param name The name of the robot.
  * @param ip The ip address of the robot.
  */
  RemoteRobot(const char* name, const char* ip);

  /**
   * Destructor.
   */
  ~RemoteRobot() {Thread<RemoteRobot>::stop(); setGlobals();}

  /**
  * The function starts the process.
  */
  void start() {Thread<RemoteRobot>::start(this, &RemoteRobot::run);}

  /**
  * The function is called to announce the termination of the process.
  */
  void announceStop();

  /**
  * The function must be called to exchange data with SimRobot.
  * It sends the motor commands to SimRobot and acquires new sensor data.
  */
  void update();

  /**
  * The function returns the name of the robot.
  * @return The name.
  */
  const char* getName() const {return name;}
};
