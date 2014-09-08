/**
* @file Representations/MotionControl/KickEngineOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"
#include "Representations/MotionControl/KickRequest.h"

/**
* @class KickEngineOutput
* A class that represents the output of the walking engine.
*/
STREAMABLE_WITH_BASE(KickEngineOutput, JointRequest,
{,
  (Pose2D) odometryOffset, /**< The body motion performed in this step. */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (bool)(true) isStable, /**< Is motion currently stable? */
  (KickRequest) executedKickRequest, /**< The kick request that is currently in execution. */
});
