/**
 * @file FieldRating.h
 *
 * Contains information about where the ball should be played to, with multiple possibilities
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(PotentialValue,
{
  PotentialValue& operator+=(const PotentialValue& other)
  {
    value += other.value;
    direction += other.direction;
    return *this;
  },

  (float)(0.f) value,
  (Vector2f)(Vector2f::Zero()) direction,
});

STREAMABLE(FieldRating,
{
  FUNCTION(PotentialValue(const float x, const float y, const bool calculateFieldDirection)) potentialFieldOnly;
  FUNCTION(void(PotentialValue& pv, const float x, const float y, bool& teammateArea, const bool calculateFieldDirection, const int passTarget)) potentialOverall;
  FUNCTION(void(PotentialValue& pv, const PotentialValue& ballNear)) removeBallNearFromTeammatePotential;
  FUNCTION(void(PotentialValue& pv, const float x, const float y, const bool calculateFieldDirection)) duelBallNearPotential;
  FUNCTION(void(PotentialValue& pv, const float x, const float y, const bool calculateFieldDirection)) getObstaclePotential;
  FUNCTION(std::vector<Vector2f>()) getPossiblePassTargets,
});
