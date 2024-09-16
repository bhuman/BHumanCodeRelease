/**
 * @file Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author Thomas Röfer
 */

#include "Settings.h"
#include "Framework/LoggingTools.h"
#include "Platform/File.h"
#include "Streaming/InStreams.h"
#include "Streaming/Output.h"

Settings::Settings(const std::string& headName, const std::string& bodyName) :
  headName(headName),
  bodyName(bodyName)
{
  InMapFile stream("settings.cfg");
  if(stream.exists())
    stream >> *this;
  else
    OUTPUT_ERROR("Could not load settings for robot \"" << headName.c_str() << "\" from settings.cfg");
  updateSearchPath();
}

Settings::Settings(const std::string& headName, const std::string& bodyName, int teamNumber, TeamColor fieldPlayerColor, TeamColor goalkeeperColor, int playerNumber, const std::string& location, const std::string& scenario, unsigned char magicNumber) :
  headName(headName),
  bodyName(bodyName),
  teamNumber(teamNumber),
  fieldPlayerColor(fieldPlayerColor),
  goalkeeperColor(goalkeeperColor),
  playerNumber(playerNumber),
  location(location),
  scenario(scenario),
  magicNumber(magicNumber)
{
  updateSearchPath();
}

Settings::Settings(const std::string& logFileName, const std::string* location, const std::string* scenario) :
  Settings("Default", "Default")
{
  InBinaryFile stream(logFileName);
  if(stream.exists())
  {
    char magicByte;
    stream >> magicByte;
    if(magicByte == LoggingTools::logFileSettings)
    {
      LoggingTools::readSettings(stream, *this);
      goto settingsRead;
    }
  }
  LoggingTools::parseName(logFileName, &headName, &bodyName, &this->scenario, &this->location, nullptr, &playerNumber);
settingsRead:
  if(location)
    this->location = *location;
  if(scenario)
    this->scenario = *scenario;
  updateSearchPath();
}

void Settings::updateSearchPath()
{
  searchPath.clear();
  searchPath.reserve(8);
  const std::string configDir = std::string(File::getBHDir()) + "/Config/";
  searchPath.push_back(configDir + "Robots/" + headName + "/Head/");
  searchPath.push_back(configDir + "Robots/" + bodyName + "/Body/");
  searchPath.push_back(configDir + "Robots/" + headName + "/" + bodyName + "/");
  if(location != "Default")
    searchPath.push_back(configDir + "Locations/" + location + "/");
  if(scenario != "Default")
    searchPath.push_back(configDir + "Scenarios/" + scenario + "/");
  searchPath.push_back(configDir + "Robots/Default/");
  searchPath.push_back(configDir + "Locations/Default/");
  searchPath.push_back(configDir + "Scenarios/Default/");
}
