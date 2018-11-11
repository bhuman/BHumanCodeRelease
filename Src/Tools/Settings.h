/**
 * @file Tools/Settings.h
 * Definition of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Streams/Enum.h"
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
    white,
    green,
    orange,
    purple,
    brown,
    gray,
  });

  std::string headName; /**< The name of this robot's head. */
  std::string bodyName; /**< The name of this robot's body. */

  static constexpr int highestValidPlayerNumber = MAX_NUM_PLAYERS; /**< No player can have a number greater than this */
  static constexpr int lowestValidPlayerNumber = 1;  /**< No player can have a number smaller than this */

  friend class ConsoleRoboCupCtrl; /**< To access settings. */

  Settings();

  static bool loadingSucceeded() { return loaded; }

private:
  static Settings settings; /**< The master settings instance. */
  static std::vector<std::string> scenarios; /**< The master scenario per team. */

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
    scenario = other.scenario.c_str();
    teamPort = other.teamPort;
    magicNumber = other.magicNumber;
    headName = other.headName.c_str(); // avoid copy-on-write
    bodyName = other.bodyName.c_str(); // avoid copy-on-write
    return *this;
  }

  /**
   * The function loads the settings from disk.
   * @return Whether the settings were loaded successfully.
   */
  bool load();

public:,
  (int) teamNumber, /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
  (TeamColor) teamColor, /**< The color of our team. Use theOwnTeamInfo.teamColor instead. */
  (int) playerNumber, /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
  (std::string) location, /**< The name of the location. */
  (std::string) scenario, /**< The name of the scenario. */
  (int) teamPort, /**< The UDP port our team uses for team communication. */
  (unsigned char) magicNumber, /**< Magic Number for the TC. Assuring no foreign packages will be processed. */
});
