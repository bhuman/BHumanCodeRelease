/**
 * @file WorldModelPrediction.h
 *
 * This file declares a class that represents data computed by modeling modules in the previous frame and
 * that is required before the excution of these modules in the current frame.
 * All positions have received odometry updates. If the ball was rolling, a dynamic update has been performed, too.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct WorldModelPrediction
 *
 * Modeling information computed in the previous frame, propagated to the current point of time.
 */
STREAMABLE(WorldModelPrediction,
{,
  (Vector2f)(Vector2f::Zero()) ballPosition,    /*< 2D position in robot coordinates, i.e. relative to the robot on the field */
  (bool)(false) ballIsValid,                    /*< true, if the previously computed ball model has actually been seen recently */
  (unsigned)(0) timeWhenBallLastSeen,           /*< as the name says */
  (Pose2f) robotPose,                           /*< the current pose of the robot in 2D field coordinates */
});
