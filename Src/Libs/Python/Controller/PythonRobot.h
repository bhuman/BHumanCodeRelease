/**
 * @file PythonRobot.h
 *
 * This file declares the class PythonRobot.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "PythonConsole.h"
#include "Framework/Debug.h"
#include "Framework/Robot.h"
#include <string>

class PythonRobot : public Robot
{
public:
  /**
   * Constructor.
   * @param settings The settings for the robot.
   * @param name The name of the robot.
   */
  PythonRobot(const Settings& settings, const std::string& name) :
    Robot(settings, name),
    console(new PythonConsole(settings, name, static_cast<Debug*>(front())))
  {
    push_back(console);
  }

  /** Triggers an update in the console thread (called from main thread). */
  void update()
  {
    console->update();
  }

private:
  PythonConsole* console; ///< The console thread for this robot.
};
