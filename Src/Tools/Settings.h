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
private:
  static Settings settings; /**< The master settings instance. */
  static bool loaded; /**< The load() of the master settings instance was called or not. */

  /**
  * Constructor for the master settings instance.
  */
  Settings(bool master);

  /**
   * Initializes the instance.
   */
  void init();

  /**
  * The function loads the settings from disk.
  * @return Whether the settings were loaded successfully.
  */
  bool load();

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
    robot = other.robot.c_str(); // avoid copy-on-write
    return *this;
  }

public:
  ENUM(TeamColor,
    blue,
    red
  );

  std::string robot; /**< The name of this robot. */
  static bool recover; /**< Start directly without the pre-initial state. */

  friend class Framework, /**< To access loaded. */

  (int)(0) teamNumber, /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
  (TeamColor)(blue) teamColor, /**< The color of our team. Use theOwnTeamInfo.teamColour instead. */
  (int)(0) playerNumber, /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
  (std::string)("Default") location, /**< The name of the location. */
  (int)(0) teamPort, /**< The UDP port our team uses for team communication. */

  // Initialization
  init();
});
