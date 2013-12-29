#pragma once

#include <vector>
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/ArmMotionRequest.h"

/**
* @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
*/
STREAMABLE(ArmMotionEngineOutput,
{
public:
  STREAMABLE(Arm,
  {,
    (bool)(false) move,                                       /**< determines whether an arm should be moved */
    (unsigned)(0) lastMovement,                               /**< Timestamp of when each arm has been moved last */
    (ArmMotionRequest, ArmMotionId)(useDefault) motion,       /**< The arm motion being executed. */
    (std::vector<float>)(4, 0) angles,                        /**< contains the target arm joints for each arm. Those values are ignored if moveArm is false */
    (std::vector<int>)(4, HardnessData::useDefault) hardness, /**< contains hardness data for this arm motion. Those values are ignored if moveArm is false*/
  }),

  (Arm[2]) arms,
});
