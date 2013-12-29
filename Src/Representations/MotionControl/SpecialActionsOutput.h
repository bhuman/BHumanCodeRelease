/**
* @file Representations/MotionControl/SpecialActionsOutput.h
* This file declares a class that represents the output of the special actions module.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Tools/Math/Pose2D.h"

/**
* @class SpecialActionsOutput
* A class that represents the output of the special actions module.
*/
STREAMABLE_WITH_BASE(SpecialActionsOutput, JointRequest,
{,
  (Pose2D) odometryOffset, /**< The body motion performed in this step. */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (bool)(false) isMotionStable, /**< Is the position of the camera directly related to the kinematic chain of joint angles? */
  (SpecialActionRequest) executedSpecialAction, /**< The special action currently executed. */
});
