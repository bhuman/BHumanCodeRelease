/**
 * @file Representations/MotionControl/ArmMotionRequest.h
 * This file declares a struct that represents the arm motions that can be requested from the robot.
 * @author Jesse Richter-Klug
 */

#pragma once

#include "ArmKeyFrameRequest.h"
#include "Tools/Arms.h"

/**
 * @struct ArmMotionRequest
 * A struct that represents the arm motions that can be requested from the robot.
 */
STREAMABLE(ArmMotionRequest,
{
  ENUM(ArmMotion,
  {,
    none, //< The motionengine provides wich also provides the leg motions, provides also the arms
    keyFrame,
  });

  ArmMotionRequest() { armMotion[Arms::left] = armMotion[Arms::right] = none; },

  (ArmMotion[Arms::numOfArms]) armMotion, /**< The selected armmotion per arm*/
  (ArmKeyFrameRequest) armKeyFrameRequest, /**< The key frame request, if it is the selected armmotion.  */
});
