/**
 * @file TorsoMatrix.h
 * Declaration of struct TorsoMatrix.
 * @author Colin Graf
 */

#pragma once

#include "Tools/Math/Pose3f.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct TorsoMatrix
 * Matrix describing the transformation from ground to the robot torso.
 */
STREAMABLE_WITH_BASE(TorsoMatrix, Pose3f,
{
  void draw(),

  (Pose3f) offset, /**< The estimated offset (including odometry) from last torso matrix to this one. (relative to the torso) */
  (bool)(false) isValid, /**< Matrix is only valid if robot is on ground. */
  (bool) leftSupportFoot, /**< Guess whether the left foot is the support foot. */
});
