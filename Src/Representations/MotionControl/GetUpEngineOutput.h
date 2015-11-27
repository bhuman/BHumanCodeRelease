/**
 * @file Representations/MotionControl/GetUpEngineOutput.h
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once

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
});
