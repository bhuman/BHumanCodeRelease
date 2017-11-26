/**
 * DiveEngineOutput
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct DiveEngineOutput
 * A struct that represents the output of the diving engine.
 */
STREAMABLE_WITH_BASE(DiveEngineOutput, JointRequest,
{,
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (Pose2f) odometryOffset,
  (int)(0) tryCounter,            /**< the number of unsuccessful tries */
});
