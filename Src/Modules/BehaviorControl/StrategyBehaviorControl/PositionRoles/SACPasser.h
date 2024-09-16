/**
 * @file SACPasser.h
 *
 * This file is an adaptation of the Passer role, for the Shared Autonomy Challenge.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "RatingRole.h"

class SACPasser : public RatingRole
{
  STREAMABLE(Parameters,
  {,
    (float)(600.f) sigmaFieldBorder, /**< standard deviation for rating dependent on distance to field border */
    (float)(50.f) sigmaCellBorder, /**< standard deviation for rating dependent on distance to cell border */
    (float)(1000.f) sigmaBall, /**< standard deviation for rating dependent on ball position. Do not stand directly near the ball as position role. */
    (float)(2000.f) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float)(0.85f) startThreshold, /**< the rating has to be at least this much better at the destination (normalized for the target pose) to start moving */
    (float)(0.60f) stopThreshold, /**< if the rating is not at least this much better at the destination (normalized for the target pose) stop moving */
    (float)(0.00001f) addWeight, /**< how much should the individual rating functions be added additionally to the multiplication */
    (float)(1500.f) minimalDistanceToTeammate, /**< the distance at which the teammate rating function should change around*/
    (float)(10.f) changeAtDistanceToTeammate, /**< how fast the rating changes around the given distance */
  });

  void preProcess() override;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const override;

  Parameters p;
};
