#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Streaming/Function.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"

/**
 * @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
 */
STREAMABLE(ArmKeyFrameGenerator,
{
  FUNCTION(void(Arms::Arm, const ArmMotionRequest&, JointRequest&, ArmMotionInfo&)) calcJoints,
});
