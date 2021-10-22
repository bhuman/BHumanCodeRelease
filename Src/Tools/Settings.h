/**
 * @file Tools/Settings.h
 * Definition of a class that provides access to settings-specific configuration directories.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/RoboCupGameControlData.h"
#include "Tools/Streams/Enum.h"
#include <string>

/**
 * @class Settings
 * The class provides access to settings-specific configuration directories.
 */
STREAMABLE(Settings,
{
public:
  /**
   * All allowed team colors. Their ordinals match the definitions of the
   * TEAM_* constants in the official header RoboCupGameControlData.h that
   * comes with the GameController.
   */
  ENUM(TeamColor,
  {,
    blue,
    red,
    yellow,
    black,
    white,
    green,
    orange,
    purple,
    brown,
    gray,
  });

  static_assert(TEAM_BLUE == blue && TEAM_RED == red && TEAM_YELLOW == yellow && TEAM_BLACK == black &&
                TEAM_WHITE == white && TEAM_GREEN == green && TEAM_ORANGE == orange &&
                TEAM_PURPLE == purple && TEAM_BROWN == brown && TEAM_GRAY == gray,
                "These macros and enums have to match!");


  /**
   * Loads all settings except for the robot name from settings.cfg.
   * @param headName The name of the robot's head.
   * @param bodyName The name of the robot's body.
   */
  Settings(const std::string& headName, const std::string& bodyName);

  /**
   * Explicitly sets all settings.
   * @param headName The name of the robot's head.
   * @param bodyName The name of the robot's body.
   * @param teamNumber The team number.
   * @param teamColor The team color.
   * @param playerNumber The player number.
   * @param location The location.
   * @param scenario The scenario.
   * @param teamPort The team port.
   * @param magicNumber The magic number.
   */
  Settings(const std::string& headName, const std::string& bodyName, int teamNumber, TeamColor teamColor, int playerNumber, const std::string& location, const std::string& scenario, int teamPort, unsigned char magicNumber);

  /**
   * Parses parts of the settings from a log file name and loads the rest from settings.cfg.
   * @param logFileName The name of the log file.
   */
  explicit Settings(const std::string& logFileName);

  static constexpr int lowestValidPlayerNumber = 1;  /**< No player can have a number smaller than this */
  static constexpr int highestValidPlayerNumber = MAX_NUM_PLAYERS; /**< No player can have a number greater than this */

  std::string headName; /**< The name of this robot's head. */
  std::string bodyName, /**< The name of this robot's body. */

  (int)(-1) teamNumber, /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
  (TeamColor)(numOfTeamColors) teamColor, /**< The color of our team. Use theOwnTeamInfo.teamColor instead. */
  (int)(-1) playerNumber, /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
  (std::string)("Default") location, /**< The name of the location. */
  (std::string)("Default") scenario, /**< The name of the scenario. */
  (int)(-1) teamPort, /**< The UDP port our team uses for team communication. */
  (unsigned char)(0) magicNumber, /**< Magic Number for team communication. Assuring no foreign packets will be processed. */
});
