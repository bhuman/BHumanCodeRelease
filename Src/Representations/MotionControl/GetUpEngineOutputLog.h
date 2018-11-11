/**
 * @file Representations/MotionControl/GetUpEngineOutputLog.h
 * @author Philip Reichenberg
 * This representation has all information that will be logged from the representation getUpEngineOutput.h
 */

#pragma once

#include "Representations/Configuration/GetupMotion.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct KickEngineOutput
 * A struct that represents the output of the walking engine.
 */
STREAMABLE(GetUpEngineOutputLog,
{,
  (int)(0) tryCounter,            /**< the number of unsuccessful tries */
  (int)(0) lineCounter,
  (GetUpMotions::GetUpMotion) name,
  (bool)(false) criticalTriggered,
  (bool)(false) optionalLine,
  (float)(0.f) theBalanceFloatY,
  (float)(0.f) theBalanceFloatX,
});
