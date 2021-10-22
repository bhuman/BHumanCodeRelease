#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Tools/Function.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(PointAtGenerator,
{
  FUNCTION(void(Arms::Arm, const ArmMotionRequest&, JointRequest&, ArmMotionInfo&)) calcJoints,
});
