/**
 * @file ControllerRobot.h
 *
 * This file declares the class ControllerRobot.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "SimulatedNao/LocalConsole.h"
#include "Framework/Debug.h"
#include "Framework/Robot.h"
#include <string>

class ConsoleRoboCupCtrl;

class ControllerRobot : public Robot
{
public:
  /**
   * Constructor.
   * @param settings The settings for the robot.
   * @param name The name of the robot.
   * @param ctrl A pointer to the controller object.
   * @param logFile The log file name to replay or an empty string to create a simulated robot.
   */
  ControllerRobot(const Settings& settings, const std::string& name, ConsoleRoboCupCtrl* ctrl, const std::string& logFile = std::string()) :
    Robot(settings, name),
    robotConsole(new LocalConsole(settings, name, ctrl, logFile, static_cast<Debug*>(front())))
  {
    push_back(robotConsole);
  }

  void update()
  {
    robotConsole->update();
  }

  RobotConsole* getRobotConsole() const { return robotConsole; }

private:
  LocalConsole* robotConsole;
};
