/**
 * @file LibTeam.h
 *
 * This file defines a representation that contains information about the team
 *
 * @author Daniel Krause
 */

#pragma once
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Streaming/Function.h"

STREAMABLE(LibTeam,
{
  /** returns the pose of a team mate in field coordinates
      @player player number of team mate */
  FUNCTION(Vector2f(int player)) getTeammatePosition;

  /** Returns the ball position estimate of the specified player */
  FUNCTION(Vector2f(int player)) getBallPosition,

  (int) strikerPlayerNumber, //player number of striker or -1 if striker was not found
  (bool) iAmClosestToBall,
  (int) numberOfNonKeeperTeammateInOwnGoalArea, /**< The number of the non keeper teammate in the goal area or -1 if there is none*/
});
