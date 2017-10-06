/**
 * @file OpponentBreakingSupporterPosition.h
 *
 * Declaration of a representation that represents the opponent breaking supporter
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include <vector>
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(OpponentBreakingSupporterPosition,
{
  void draw() const,

  (bool)(false) isValid,
  (Obstacle) asObstacle, ///< this should be a global one
});
