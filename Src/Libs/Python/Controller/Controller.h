/**
 * @file Controller.h
 *
 * This file declares a class that controls a set of robots.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "PythonRobot.h"
#include "Framework/Settings.h"
#include <memory>
#include <string>
#include <vector>

class Controller final
{
public:
  /** Constructor. */
  Controller();

  /** Destructor. Assumes that \c stop has been called before. */
  ~Controller();

  /** Starts all robots. */
  void start();

  /** Stops all robots. */
  void stop();

  /** Performs a single step in the robots. */
  void update();

  /**
   * Adds a player to the controller.
   * @param name The name of the player (only influences thread names but not the config search path).
   * @param teamNumber The team number of the player.
   * @param fieldPlayerColor The jersey color of the field players in this player's team.
   * @param goalkeeperColor The jersey color of the goalkeeper in this player's team.
   * @param playerNumber The (jersey) number of the player.
   * @param location The location in the configuration file search path.
   * @param scenario The scenario in the configuration file search path.
   */
  void addPlayer(const std::string& name, int teamNumber, Settings::TeamColor fieldPlayerColor, Settings::TeamColor goalkeeperColor, int playerNumber, const std::string& location, const std::string& scenario);

private:
  std::vector<std::unique_ptr<PythonRobot>> robots; /**< List of registered robots. */
  static Controller* controller; /**< The only \c Controller instance in this process. */
};
