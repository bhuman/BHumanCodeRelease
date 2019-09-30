/**
 * @file Representations/MotionControl/GetUpEngineOutputLog.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Motion/EngineState.h"
#include "Tools/Motion/GetUpMotion.h"

//First Vector value is x, second is y
STREAMABLE(MotionLineInfoLog,
{,
  (Vector2f)(0.f, 0.f) comDif,
  (Vector2f)(0.f, 0.f) balanceValue,
});

STREAMABLE(CurrentLineInfoLog,
{,
  (float)(0.f) ratio,
  (EngineStates::EngineState) state,
  (GetUpMotions::GetUpMotion) name,
  (int)(0) lineCounter,
  (int)(0) maxCounter,
  (float)(0.f) framesTillEnd,
  (bool)(false) isMirror,
  (bool)(false) isInOptionalLine,
});

/**
 * This Representation saves all important Values of GetUpEngineOutput
 */
STREAMABLE(GetUpEngineOutputLog,
{
  void draw(),
       (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
       (Pose2f) odometryOffset,
       (int)(0) tryCounter,            /**< the number of unsuccessful tries */
       (CurrentLineInfoLog) currentLine,
       (MotionLineInfoLog) lineInfo,
       (bool)(false) recoverArmLeftDone,
       (bool)(false) recoverArmRightDone,
       (bool)(false) recoverSideDone,
       (bool)(false) failBack,
       (bool)(false) failFront,
       (bool)(false) errorTriggered,
       (int)(0) waitTime,
});
