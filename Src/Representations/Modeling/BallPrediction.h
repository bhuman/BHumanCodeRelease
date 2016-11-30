/**
 * @file BallPrediction.h
 *
 * The file declares a class that represents the position of the ball in
 * robot coordinates, as estimated by BallLocator in the previous frame,
 * propagated to the current point of time.
 *
 * @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Math/Eigen.h"

/**
 * @struct BallPrediction
 *
 * The position of the ball in robot coordinates, as estimated by BallLocator
 * in previous frame, propagated to the current point of time.
 */
STREAMABLE(BallPrediction,
{,
  (Vector2f)(Vector2f::Zero()) position,
  (bool)(false) isValid,
  (unsigned)(0) timeWhenLastSeen,
});
