/**
 * @file Representations/MotionControl/ArmMotionInfo.h
 * This file declares a struct that represents the arm motions that can be requested from the robot.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(ArmMotionInfo, ArmMotionRequest,
{
  bool isKeyframeMotion(Arms::Arm arm, ArmKeyFrameRequest::ArmKeyFrameId motion) const
  {
    return armMotion[arm] == keyFrame && armKeyFrameRequest.arms[arm].motion == motion;
  },

  (ENUM_INDEXED_ARRAY(bool, Arms::Arm))({false, false}) isFree,
});
