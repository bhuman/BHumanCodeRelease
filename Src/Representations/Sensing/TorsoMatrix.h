/**
 * @file TorsoMatrix.h
 * Declaration of struct TorsoMatrix.
 * @author Colin Graf
 */

#pragma once

#include "Math/Pose3f.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct TorsoMatrix
 * Matrix describing the transformation from ground to the robot torso.
 */
STREAMABLE_WITH_BASE(TorsoMatrix, Pose3f,
{
  TorsoMatrix() = default;

  void draw(),

  (bool)(false) isValid, /**< Matrix is only valid if robot is on ground. */
});
