/**
 * @file TorsoMatrix.h
 * Declaration of struct TorsoMatrix.
 * @author Colin Graf
 */

#pragma once

#include "Math/SE3fWithCov.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"

/**
 * @struct TorsoMatrix
 * Matrix describing the transformation from ground to the robot torso.
 */
STREAMABLE_WITH_BASE(TorsoMatrix, SE3WithCov,
{
  TorsoMatrix() = default;

  void draw(),

  (bool)(false) isValid, /**< Matrix is only valid if robot is on ground. */
});
