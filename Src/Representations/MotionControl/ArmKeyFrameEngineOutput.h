#pragma once

#include <vector>
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"

/**
 * @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
 */
STREAMABLE_WITH_BASE(ArmKeyFrameEngineOutput, ArmJointRequest,
{
  STREAMABLE(Arm,
  {,
    (bool)(true) isFree,                                    /**< The arm is ready to release by key fram engine */
    (ArmKeyFrameRequest::ArmKeyFrameId)(ArmKeyFrameRequest::useDefault) motion, /**< The arm motion being executed. */
  }),

  (ENUM_INDEXED_ARRAY(Arm, Arms::Arm)) arms,
});
