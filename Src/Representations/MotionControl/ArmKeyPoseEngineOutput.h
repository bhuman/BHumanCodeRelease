#pragma once

#include <vector>
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyPoseRequest.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
STREAMABLE_WITH_BASE(ArmKeyPoseEngineOutput, JointRequest,
{
  STREAMABLE(Arm,
  {,
    (ArmKeyPoseRequest::ArmKeyPoseMotionId)(ArmKeyPoseRequest::nullPose) pose, /**< The arm motion being executed. */
  }),

  (ENUM_INDEXED_ARRAY(Arm, Arms::Arm)) arms,
});
