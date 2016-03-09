/**
 * @file Tools/Settings.h
 * Definition of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @class Settings
 * The class provides access to settings-specific configuration directories.
 */
STREAMABLE(Settings,
{
public:
  ENUM(TeamColor,
  {,
    blue,
    red,
    yellow,
    black,
  });

  std::string robotName; /**< The name of this robot. */
  std::string bodyName; /**< The name of this robot's body. */

  static bool recover; /**< Start directly without the pre-initial state. */

  static constexpr int highestValidPlayerNumber = 6; /**< No player can have a number greater than this */
  static constexpr int lowestValidPlayerNumber = 1;  /**< No player can have a number smaller than this */
  bool isGoalkeeper;            /**< Is this robot the goaliekeeper? */
  bool isDropInGame = false;    /**< Is this a normal game or a dropin game? */
  bool isCornerChallenge = false;
  bool isCarpetChallenge = false;
  bool isRealBallChallenge = false;

  friend class Framework; /**< To access loaded. */


  Settings();

  static bool loadingSucceeded() { return loaded; }

private:
  static Settings settings; /**< The master settings instance. */
  static bool loaded; /**< True if the load() of the master settings instance succeeded. */

  /**
   * Constructor for the master settings instance.
   */
  Settings(bool master);

  /**
   * Assignment operator
   * @param other The other settings that is assigned to this one
   * @return A reference to this object after the assignment.
   */
  Settings& operator=(const Settings& other)
  {
    teamNumber = other.teamNumber;
    teamColor = other.teamColor;
    playerNumber = other.playerNumber;
    location = other.location.c_str(); // avoid copy-on-write
    teamPort = other.teamPort;
    robotName = other.robotName.c_str(); // avoid copy-on-write
    bodyName = other.bodyName.c_str(); // avoid copy-on-write
    isGoalkeeper = other.isGoalkeeper;
    isDropInGame = other.isDropInGame;
    isCornerChallenge = other.isCornerChallenge;
    isCarpetChallenge = other.isCarpetChallenge;
    isRealBallChallenge = other.isRealBallChallenge;
    return *this;
  }

  /**
   * The function loads the settings from disk.
   * @return Whether the settings were loaded successfully.
   */
  bool load();

public:
  ,
  (int)(0) teamNumber, /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
  (TeamColor)(blue) teamColor, /**< The color of our team. Use theOwnTeamInfo.teamColor instead. */
  (int)(0) playerNumber, /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
  (std::string)("Default") location, /**< The name of the location. */
  (int)(0) teamPort, /**< The UDP port our team uses for team communication. */
});
