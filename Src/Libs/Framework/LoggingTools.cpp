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
  stream << unsigned(1);
  stream << settings.headName;
  stream << settings.bodyName;
  stream << settings.playerNumber;
  stream << settings.location;
  stream << settings.scenario;
}

void LoggingTools::readSettings(In& stream, Settings& settings)
{
  unsigned version;
  stream >> version;
  if(version == 1)
  {
    stream >> settings.headName;
    stream >> settings.bodyName;
    stream >> settings.playerNumber;
    stream >> settings.location;
    stream >> settings.scenario;
  }
  else
    FAIL("Unknown settings version " << version << ".");
}

void LoggingTools::skipSettings(In& stream)
{
  unsigned version;
  stream >> version;
  if(version == 1)
  {
    decltype(Settings::headName) string;
    static_assert(std::is_same<decltype(Settings::headName), decltype(Settings::bodyName)>::value);
    static_assert(std::is_same<decltype(Settings::headName), decltype(Settings::location)>::value);
    static_assert(std::is_same<decltype(Settings::headName), decltype(Settings::scenario)>::value);
    decltype(Settings::playerNumber) number;
    stream >> string;
    stream >> string;
    stream >> number;
    stream >> string;
    stream >> string;
  }
  else
    FAIL("Unknown settings version " << version << ".");
}

std::string LoggingTools::createName(const std::string& headName, const std::string& bodyName, const std::string& scenario,
                                     const std::string& location, const std::string& identifier, int playerNumber,
                                     const std::string& suffix)
{
  return headName + "_" + bodyName + "_" + scenario + "_" + location + "__" + identifier + "_" + std::to_string(playerNumber) +
         (!suffix.empty() ? "_" + suffix : "");
}

void LoggingTools::parseName(const std::string& logfileName, std::string* headName, std::string* bodyName,
                             std::string* scenario, std::string* location, std::string* identifier, int* playerNumber, std::string* suffix)
{
  static std::regex hbslrc22("([A-Za-z]*)_([A-Za-z]*)_([A-Za-z0-9]*)_([A-Za-z0-9]*_Field[A-Z])__"); // HACK: (RC2022) regex for head name, body name, scenario and location
  static std::regex hbsl("([A-Za-z]*)_([A-Za-z]*)_([A-Za-z0-9]*)_([A-Za-z0-9]*)__"); // regex for head name, body name, scenario and location
  static std::regex ip("__([A-Za-z0-9_-]*)_([0-9][0-9]*)(_\\([0-9][0-9]*\\)){0,1}\\."); // regex for identifier and player number
  std::smatch match;

  if(std::regex_search(logfileName, match, hbslrc22))
  {
    ASSERT(match.size() == 5);
    if(headName)
      *headName = match[1];
    if(bodyName)
      *bodyName = match[2];
    if(scenario)
      *scenario = match[3];
    if(location)
      *location = match[4];
  }
  else if(std::regex_search(logfileName, match, hbsl))
  {
    ASSERT(match.size() == 5);
    if(headName)
      *headName = match[1];
    if(bodyName)
      *bodyName = match[2];
    if(scenario)
      *scenario = match[3];
    if(location)
      *location = match[4];
  }

  if(std::regex_search(logfileName, match, ip))
  {
    ASSERT(match.size() == 4);
    if(identifier)
      *identifier = match[1];
    if(playerNumber)
      *playerNumber = std::stoi(match[2]);
    if(suffix)
      *suffix = match[3].matched ? match[3].str().substr(1) : "";
  }
}
