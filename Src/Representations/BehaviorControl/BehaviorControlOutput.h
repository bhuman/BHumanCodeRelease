/**
 * @file BehaviorControlOutput.h
 * Declaration of class BehaviorControlOutput
 *
 * @author Max Risler
 */

#pragma once

#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/ActivationGraph.h"

/**
 * A class collecting the output from the behavior control module
 */
STREAMABLE(BehaviorControlOutput,
{,
  (ArmMotionRequest) armMotionRequest,
  (MotionRequest) motionRequest,
  (HeadMotionRequest) headMotionRequest,
  (OwnTeamInfo) ownTeamInfo,
  (RobotInfo) robotInfo,
  (GameInfo) gameInfo,
  (BehaviorStatus) behaviorStatus,
  (BehaviorLEDRequest) behaviorLEDRequest,
  (ActivationGraph) executionGraph,
});
