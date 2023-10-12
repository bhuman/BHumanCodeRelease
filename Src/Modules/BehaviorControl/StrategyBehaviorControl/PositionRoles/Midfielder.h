/**
 * @file Midfielder.h
 *
 * This file declares the Midfielder role.
 * It tries to maximize a rating function.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "RatingRole.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Math/RingBufferWithSum.h"

class Midfielder : public RatingRole
{
  STREAMABLE(Parameters,
  {,
    (float)(2000.f) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float)(1000.f) sigmaTeam, /**< standard deviation for rating dependent on distance to teammates */
    (float)(600.f) sigmaBorder, /**< standard deviation for rating dependent on distance to field border */
    (float)(2500.f) ballDistance, /**< preferred distance to ball */
    (float)(1000.f) sigmaBall, /**< standard deviation for rating dependent on distance to ball */
    (float)(0.8f) startThreshold, /**< the rating has to be at least this much better (normalised to the new found one) at the destination to start moving */
    (float)(0.3f) stopThreshold, /**< if the rating is not at least this much better (normalised to the new found one) at the destination stop moving */
    (float)(0.5f) baseShiftX, /**< factor by which the base pose is moved toward the X position of the ball */
    (float)(0.5f) maxBaseShilftY, /**< factor by which the base pose is moved toward the Y position of the ball at maximum */
    (float)(1000.f) smoothBaseShiftYDistence, /**< distance from the mid line in which the base shift in Y direction is smoothed linearly */
    (float)(0.5f) minBallRating, /**< minimal value of the rating depending on the distance to the ball */
    (float)(0.5f) minGoalRating, /**< minimal value of the goal rating */
  });

  void preProcess() override;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const override;

  Parameters p;
};
