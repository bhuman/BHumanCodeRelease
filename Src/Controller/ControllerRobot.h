/**
 * @file ControllerRobot.h
 *
 * This file declares the class ControllerRobot.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Controller/LocalRobot.h"
#include "Threads/Debug.h"
#include "Tools/Framework/Robot.h"

class ControllerRobot : public Robot
{
public:
  /**
   * Constructor.
   * @param settings The settings for the robot.
   * @param name The name of the robot.
   */
  ControllerRobot(const Settings& settings, const std::string& name) :
    Robot(settings, name),
    robotThread(new LocalRobot(settings, name, static_cast<Debug*>(front())))
  {
    push_back(robotThread);
  }

  void update()
  {
    robotThread->update();
  }

  RobotConsole* getRobotThread() const { return robotThread; }

private:
  LocalRobot* robotThread;
};
