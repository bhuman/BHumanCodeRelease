/*
 * @file Odometer.h
 *
 * Some additional odometry information
 *
 * @author Tim Laue
 * @author Marcel Steinbeck
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(Odometer,
{
  /** Verifies that the representation contains valid values. */
  void verify() const,

  (float)(10000.f) distanceWalked, /**< Total distance walked since start of B-Human software */
  (Pose2f) odometryOffset, /**< Odometry difference since last frame */
  (Matrix3f) odometryOffsetCovariance, /**< Approximation to the covariance of an odometry offset in the form: [x, y, angle]^T * [x, y, angle]. */
});
