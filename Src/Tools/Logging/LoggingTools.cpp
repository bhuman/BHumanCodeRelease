/**
 * @file LoggingTools.cpp
 *
 * This file implements the functions from the LoggingTools namespace.
 *
 * @author Arne Hasselbring
 */

#include "LoggingTools.h"
#include "Platform/BHAssert.h"
#include <regex>

std::string LoggingTools::createName(const std::string& prefix, const std::string& headName, const std::string& bodyName,
                                     const std::string& scenario, const std::string& location, const std::string& identifier,
                                     int playerNumber, const std::string& suffix)
{
  return prefix + (prefix.empty() || prefix[prefix.size() - 1] == '/' ? "" : "_") + headName + "_" + bodyName + "_" + scenario + "_" + location + "__" + identifier + "_" + std::to_string(playerNumber) +
         (!suffix.empty() ? "_" + suffix : "");
}

void LoggingTools::parseName(const std::string& logfileName, std::string* prefix, std::string* headName, std::string* bodyName,
                             std::string* scenario, std::string* location, std::string* identifier, int* playerNumber, std::string* suffix)
{
  static std::regex thbsl("([A-Za-z_]*)_([A-Za-z]*)_([A-Za-z]*)_([A-Za-z0-9]*)_([A-Za-z0-9]*)__"); // regex for thread, head name, body name, scenario and location
  static std::regex hbsl("([A-Za-z]*)_([A-Za-z]*)_([A-Za-z0-9]*)_([A-Za-z0-9]*)__"); // regex for head name, body name, scenario and location
  static std::regex thb("([A-Za-z_]*)_([A-Za-z]*)_([A-Za-z]*)__"); // regex for only thread, head name and body name
  static std::regex ip("__([A-Za-z0-9_-]*)_([0-9][0-9]*)(_\\([0-9][0-9]*\\)){0,1}\\."); // regex for identifier and player number
  std::smatch match;

  if(std::regex_search(logfileName, match, thbsl))
  {
    ASSERT(match.size() == 6);
    if(prefix)
      *prefix = match[1];
    if(headName)
      *headName = match[2];
    if(bodyName)
      *bodyName = match[3];
    if(scenario)
      *scenario = match[4];
    if(location)
      *location = match[5];
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
  else if(std::regex_search(logfileName, match, thb))
  {
    ASSERT(match.size() == 4);
    if(prefix)
      *prefix = match[1];
    if(headName)
      *headName = match[2];
    if(bodyName)
      *bodyName = match[3];
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
