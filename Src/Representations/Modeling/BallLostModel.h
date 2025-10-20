/**
 * @file BallLostModel.h
 *
 * This representation holds information of the ball, when it was seen last as an alternate position direction
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(BallLostModel,
{
  /** Draws the estimate on the field */
  void draw() const,

  (Vector2f)(Vector2f::Zero()) relativeAlternateBallPosition, /**< Equals the ball model if no collision with the feet occurred. Otherwise it used the previous ball position and applied an own odometry update. */
  (Angle)(0_deg) relativeAlternateBallDirectionWhenLastSeen, /**< The relative ball direction when last seen. */
});
