#pragma once

#include "Streaming/AutoStreamable.h"
#include <string>
#include <vector>

class Context;

STREAMABLE(Robot,
{
  std::string getBestIP(const Context& context) const;

  static std::vector<Robot> getRobots(),

  (std::string) lan,
  (std::string) wlan,
  (std::string) name,
});
