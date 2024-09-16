/**
 * @file SentTeamMessage.h
 *
 * This file defines a representation that  contains some of the representations
 * that were last sent in a team message.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/InitialToReady.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Communication/RobotStatus.h"

STREAMABLE(SentTeamMessage,
{,
  (BallModel) theBallModel,
  (BehaviorStatus) theBehaviorStatus,
  (FrameInfo) theFrameInfo,
  (RobotPose) theRobotPose,
  (RobotStatus) theRobotStatus,
  (StrategyStatus) theStrategyStatus,
  (Whistle) theWhistle,
  (IndirectKick) theIndirectKick,
  (InitialToReady) theInitialToReady,
});
