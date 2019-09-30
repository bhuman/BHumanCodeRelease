/**
 * @file Representations/MotionControl/GetUpEngineOutput.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Motion/EngineState.h"
#include "Tools/Motion/GetUpMotion.h"

//First Vector value is x, second is y
STREAMABLE(MotionLineInfo,
{,
  (Vector2f)(0.f, 0.f) comDif,
  (Vector2f)(0.f, 0.f) balanceValue,
});

STREAMABLE(CurrentLineInfo,
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
 * @struct GetUpEngineOutput
 * A struct that represents the output of the getUpEngine.
 */
STREAMABLE_WITH_BASE(GetUpEngineOutput, JointRequest,
{
  void draw() const,
       (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
       (Pose2f) odometryOffset,
       (int)(0) tryCounter,            /**< the number of unsuccessful tries */
       (CurrentLineInfo) currentLine,
       (MotionLineInfo) lineInfo,
       (bool)(false) recoverArmLeftDone,
       (bool)(false) recoverArmRightDone,
       (bool)(false) recoverSideDone,
       (bool)(false) failBack,
       (bool)(false) failFront,
       (bool)(false) errorTriggered,
       (Pose3f) supportCenterInTorso, //getting removed after BA is writting down.
       (Pose3f) comInTorso,
       (Pose3f) gyroInTorso,
       (std::vector<Pose3f>) supportPolygon,
       (int)(0) waitTime,
});
