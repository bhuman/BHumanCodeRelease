/**
 * @file Representations/MotionControl/GetUpEngineOutput.h
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/GetupMotion.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct KickEngineOutput
 * A struct that represents the output of the walking engine.
 */
STREAMABLE_WITH_BASE(GetUpEngineOutput, JointRequest,
{,
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (Pose2f) odometryOffset,
  (int)(0) tryCounter,            /**< the number of unsuccessful tries */
  (int)(0) lineCounter, /**< line of the current motion */
  (bool)(false) failBack,
  (bool)(false) failFront,
  (GetUpMotions::GetUpMotion) name, /**< current motion */
  (bool)(false) criticalTriggered, /**< motion needs to stop */
  (bool)(false) optionalLine, /**< are we in a optional line? */
  (float)(0.f) theBalanceFloatY,/**< the balanceValue, that will get added on top of the joints for balancing forward and backward */
  (float)(0.f) theBalanceFloatX, /**< the balance value, that will get added on top of the joints for balancing sideways */
});
