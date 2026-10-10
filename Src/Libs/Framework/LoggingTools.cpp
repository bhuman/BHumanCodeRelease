/**
 * @file LoggingTools.cpp
 *
 * This file implements the functions from the LoggingTools namespace.
 *
 * @author Arne Hasselbring
 */

#include "LoggingTools.h"
#include "Framework/Settings.h"
#include "Platform/BHAssert.h"
#include "Streaming/InOut.h"
#include <regex>
#include <type_traits>

void LoggingTools::writeSettings(Out& stream, const Settings& settings)
{
  stream << unsigned(3);
  stream << settings.robotName;
  stream << settings.playerNumber;
  stream << settings.location;
  stream << settings.scenario;
  stream << settings.teamNumber;
}

void LoggingTools::readSettings(In& stream, Settings& settings)
{
  unsigned version;
  stream >> version;
  if(version >= 1 && version <= 3)
  {
    stream >> settings.robotName;
    if(version < 3)
    {
      std::string bodyName;
      stream >> bodyName;
    }
    stream >> settings.playerNumber;
    stream >> settings.location;
    stream >> settings.scenario;
    if(version >= 2)
      stream >> settings.teamNumber;
  }
  else
    FAIL("Unknown settings version " << version << ".");
}

void LoggingTools::skipSettings(In& stream)
{
  unsigned version;
  stream >> version;
  if(version >= 1 && version <= 3)
  {
    decltype(Settings::robotName) string;
    decltype(Settings::playerNumber) number;
    static_assert(std::is_same<decltype(Settings::robotName), decltype(Settings::location)>::value);
    static_assert(std::is_same<decltype(Settings::robotName), decltype(Settings::scenario)>::value);
    static_assert(std::is_same<decltype(Settings::playerNumber), decltype(Settings::teamNumber)>::value);
    stream >> string;
    if(version < 3)
      stream >> string;
    stream >> number;
    stream >> string;
    stream >> string;
    if(version >= 2)
      stream >> number;
  }
  else
    FAIL("Unknown settings version " << version << ".");
}

std::string LoggingTools::createName(const std::string& robotName, const std::string& scenario, const std::string& location,
                                     const std::string& identifier, int playerNumber, const std::string& suffix)
{
  return robotName + "_" + scenario + "_" + location + "__" + identifier + "_" + std::to_string(playerNumber) +
         (!suffix.empty() ? "_" + suffix : "");
}
