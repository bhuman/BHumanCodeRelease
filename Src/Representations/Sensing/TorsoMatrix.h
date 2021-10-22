/**
 * @file TorsoMatrix.h
 * Declaration of struct TorsoMatrix.
 * @author Colin Graf
 */

#pragma once

#include "Tools/Math/Pose3f.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"

/**
 * @struct TorsoMatrix
 * Matrix describing the transformation from ground to the robot torso.
 */
STREAMABLE_WITH_BASE(TorsoMatrix, Pose3f,
{
  TorsoMatrix() = default;

  void setTorsoMatrix(const InertialData& theInertialData, const RobotModel& theRobotModel, const GroundContactState& theGroundContactState);
  void draw(),

  (bool)(false) isValid, /**< Matrix is only valid if robot is on ground. */
});
