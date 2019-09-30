/**
 * @file Tools/Framework/Robot.h
 *
 * This file declares the class Robot as a list of threads.
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Framework/ThreadFrame.h"
#include "Tools/Logging/Logger.h"

class RobotConsole;

/**
 * @class Robot
 *
 * The class implements a robot as a list of threads.
 */
class Robot : public ThreadList
{
private:
#ifdef TARGET_SIM
  RobotConsole* robotThread = nullptr; /**< A pointer to the thread that simulates the physical robots. */
#endif
  std::string name; /**< The name of the robot. */
  Logger* logger; /**< The logger for data from all threads. */

public:
  /**
   * The constructor.
   * @param name The name of the robot.
   */
  Robot(const std::string& name);

  /** Destructor. */
  ~Robot() {delete logger;}

  /**
   * The function returns the name of the robot.
   * @return The name of the robot.
   */
  const std::string& getName() const { return name; }

#ifdef TARGET_SIM
  /**
   * The function updates all sensors and sends motor commands to SimRobot.
   */
  void update();

  /**
   * The function returns a pointer to the thread that simulates the physical robots.
   * @return The pointer to the thread.
   */
  RobotConsole* getRobotThread() const { return robotThread; }
#endif
};
