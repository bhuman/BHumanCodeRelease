/**
 * @file Robots.h
 *
 * This file defines a representation that maps robot names to robot types.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Settings.h"

STREAMABLE(Robots,
{
  STREAMABLE(RobotId,
  {,
    (std::string) name,
    (Settings::RobotType) robotType,
  }),

  (std::vector<RobotId>) robotsIds,
});
