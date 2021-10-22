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
#include <string>

struct Settings;

/**
 * @class Robot
 *
 * The class implements a robot as a list of threads.
 */
class Robot : public ThreadList
{
private:
  std::string name; /**< The name of the robot. */
  Logger* logger; /**< The logger for data from all threads. */

public:
  /**
   * The constructor.
   * @param settings The settings for the robot.
   * @param name The name of the robot.
   */
  Robot(const Settings& settings, const std::string& name);

  /** Destructor. */
  ~Robot() { delete logger; }

  /**
   * The function returns the name of the robot.
   * @return The name of the robot.
   */
  const std::string& getName() const { return name; }
};
