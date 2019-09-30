#pragma once

#include <climits>
#include <map>
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

class Context;
class Session;
class Team;

STREAMABLE(Robot,
{
  std::string getBestIP(const Context& context) const;

  static std::vector<Robot> getRobots();
  static void initRobotsByName(std::map<std::string, Robot*>& robotsByName),

  (std::string) lan,
  (std::string) wlan,
  (std::string) name,
});
