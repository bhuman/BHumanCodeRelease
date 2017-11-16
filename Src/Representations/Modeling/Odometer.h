/*
 * @file Odometer.h
 *
 * Some additional odometry information
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author marcel
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Odometer,
{,
  (float)(10000.f) distanceWalked, /** Total distance walked since start of B-Human software */
  (Pose2f) odometryOffset, /** Odometry difference since last frame */
  (Matrix3f) odometryOffsetCovariance, /**< Approximation to the covariance of an odometry offset in the form: [x, y, angle]^T * [x, y, angle]. */
});
