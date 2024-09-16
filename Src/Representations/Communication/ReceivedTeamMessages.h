/**
 * @file ReceivedTeamMessages.h
 *
 * This file declares a representation that contains the team messages
 * that have been received in a single frame.
 *
 * @author Arne Hasselbrng
 */

#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/InitialToReady.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Communication/RobotStatus.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(ReceivedTeamMessage,
{,
  (int)(-1) number,

  (RobotStatus) theRobotStatus,
  (RobotPose) theRobotPose,
  (BallModel) theBallModel,
  (FrameInfo) theFrameInfo,
  (BehaviorStatus) theBehaviorStatus,
  (Whistle) theWhistle,
  (StrategyStatus) theStrategyStatus,
  (IndirectKick) theIndirectKick,
  (InitialToReady) theInitialToReady,
});

STREAMABLE(ReceivedTeamMessages,
{,
  (std::vector<ReceivedTeamMessage>) messages, /**< An unordered(!) list of all team messages that have been received in this frame. */
  (unsigned)(0) unsynchronizedMessages,        /**< The number of received team messages in this frame that were rejected due to lack of known clock offset. */
});
