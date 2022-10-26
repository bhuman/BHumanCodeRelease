#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Streaming/Function.h"
#include "RobotParts/Arms.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(PointAtGenerator,
{
  FUNCTION(void(Arms::Arm, const ArmMotionRequest&, JointRequest&, ArmMotionInfo&)) calcJoints,
});
