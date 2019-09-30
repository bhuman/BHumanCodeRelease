/**
 * @file Representations/MotionControl/ArmMotionRequest.h
 * This file declares a struct that represents the arm motions that can be requested from the robot.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "ArmKeyFrameRequest.h"
#include "ArmKeyPoseRequest.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct ArmMotionRequest
 * A struct that represents the arm motions that can be requested from the robot.
 */
STREAMABLE(ArmMotionRequest,
{
  ENUM(ArmRequest,
  {,
    none, /**< The motionengine provides which also provides the leg motions, provides also the arms */
    clear,
    keyFrame,
    keyPose,
    pointAt,
  });

  ArmMotionRequest() { armMotion[Arms::left] = armMotion[Arms::right] = none; },

  (ENUM_INDEXED_ARRAY(ArmMotionRequest::ArmRequest, Arms::Arm)) armMotion, /**< The selected armmotion per arm */
  (ArmKeyFrameRequest) armKeyFrameRequest, /**< The key frame request, if it is the selected armmotion. */
  (ArmKeyPoseRequest) armKeyPoseRequest, /**< The key pose request, if it is the selected armmotion. */
  (Vector3f)(Vector3f::Zero()) pointToPointAt, /**< The point to point at, if selected */
});

STREAMABLE_WITH_BASE(ArmMotionInfo, ArmMotionRequest,
{,
});
