/**
 * @file Tools/Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author Thomas Röfer
 */

#include "Settings.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Logging/LoggingTools.h"
#include "Tools/Streams/InStreams.h"

Settings::Settings(const std::string& headName, const std::string& bodyName) :
  headName(headName),
  bodyName(bodyName)
{
  InMapFile stream("settings.cfg");
  if(stream.exists())
    stream >> *this;
  else
    OUTPUT_ERROR("Could not load settings for robot \"" << headName.c_str() << "\" from settings.cfg");
}

Settings::Settings(const std::string& headName, const std::string& bodyName, int teamNumber, TeamColor teamColor, int playerNumber, const std::string& location, const std::string& scenario, int teamPort, unsigned char magicNumber) :
  headName(headName),
  bodyName(bodyName),
  teamNumber(teamNumber),
  teamColor(teamColor),
  playerNumber(playerNumber),
  location(location),
  scenario(scenario),
  teamPort(teamPort),
  magicNumber(magicNumber)
{}

Settings::Settings(const std::string& logFileName) :
  Settings("Default", "Default")
{
  LoggingTools::parseName(logFileName, nullptr, &headName, &bodyName, &scenario, &location, nullptr, &playerNumber);
}
