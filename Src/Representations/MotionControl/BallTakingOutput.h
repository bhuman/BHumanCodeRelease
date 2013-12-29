/**
* @file BallTakingOutput.h
* @author Thomas Münder
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"

STREAMABLE_WITH_BASE(BallTakingOutput, JointRequest,
{,
  (Pose2D) odometryOffset,
  (bool)(true) isLeavingPossible,
  (bool)(false) isTakable,
});
