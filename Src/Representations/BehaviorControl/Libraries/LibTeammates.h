/**
 * @file LibTeammates.h
 *
 * A representation that contains information about teammate positions
 *
 * @author Lukas Malte Monnerjahn
 */

#pragma once
#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(LibTeammates,
{
  /** Returns the ball position estimate of the current striker
      TODO: Better implementation (current one sucks) or deletion!
   */
  FUNCTION(Vector2f()) getStrikerBallPosition,

  (int) nonKeeperTeammatesInOwnGoalArea, /**< How many non keeper teammates are in the own goal area */
});
