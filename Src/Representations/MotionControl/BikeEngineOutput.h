/**
* @file Representations/MotionControl/BikeEngineOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/Pose2D.h"
#include "Representations/MotionControl/BikeRequest.h"

/**
* @class BikeEngineOutput
* A class that represents the output of the walking engine.
*/
STREAMABLE_WITH_BASE(BikeEngineOutput, JointRequest,
{,
  (Pose2D) odometryOffset, /**< The body motion performed in this step. */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (bool)(true) isStable, /**< Is motion currently stable? */
  (BikeRequest) executedBikeRequest, /**< The bike request that is currently in execution. */
});
