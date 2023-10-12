/**
 * @file Forward.h
 *
 * This file declares the forward role.
 * It tries to maximize a rating function in order to receive a pass and score a goal.
 *
 * @author Yannik Meinken
 */

#pragma once

#include "RatingRole.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Math/RingBufferWithSum.h"

class Forward : public RatingRole
{
  STREAMABLE(Parameters,
  {,
    (float)(2000.f) sigmaBase, /**< standard deviation for rating dependent on base position */
    (float)(0.85f) startThreshold, /**< the rating has to be at least this much better at the destination (normalized for the target pose) to start moving */
    (float)(0.60f) stopThreshold, /**< if the rating is not at least this much better at the destination (normalized for the target pose) stop moving */
    (float)(0.00001f) addWeight, /**< how much should the individual rating functions be added additionally to the multiplication */
  });

  void preProcess() override;

  //computes the rating for a given point
  float rating(const Vector2f& pos) const override;

  Parameters p;
};
