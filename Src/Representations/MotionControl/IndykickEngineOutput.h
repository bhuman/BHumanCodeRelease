/**
* @file Representations/MotionControl/IndykickEngineOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author Felix Wenk
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Math/Vector.h"
#include "Representations/MotionControl/IndykickRequest.h"

/**
* @class IndykickEngineOutput
* A class that represents the output of the balancing kick engine.
*/
STREAMABLE_WITH_BASE(IndykickEngineOutput, JointRequest,
{,
  (Pose2D) odometryOffset, /**< The body motion performed in this step. */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (IndykickRequest) executedIndykickRequest, /**< The kick request that is currently in execution. */
  (Vector2f) ref, /**< Reference point in the support foot relative to the support foot. This should be the ZMP. */
  (float[RAnkleRoll - RHipYawPitch + 1]) notGyroCorrectedRightLegAngles,
  (float[LAnkleRoll - LHipYawPitch + 1]) notGyroCorrectedLeftLegAngles,
  (bool)(false) useGyroCorrection,
});
