/**
 * @file Representations/MotionControl/KickEngineOutput.h
 * This file declares a struct that represents the output of modules generating motion.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"

/**
 * @struct KickEngineOutput
 * A struct that represents the output of the kick engine.
 */
STREAMABLE_WITH_BASE(KickEngineOutput, JointRequest,
{,
  (Pose2f) odometryOffset, /**< The body motion performed in this step. */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (bool)(true) isStable, /**< Is motion currently stable? */
  (bool)(false) inAction, /**< Is a motion chain currently stable? */
  (bool)(true) wasValid, /**< Is a motion chain currently stable? */
  (KickRequest) executedKickRequest, /**< The kick request that is currently in execution. */
  (Pose3f) desiredFootPosition, /**< Which Position shoudl be reached */
});
