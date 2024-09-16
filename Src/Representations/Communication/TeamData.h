/**
 * @file TeamData.h
 *
 * This representation should soon be obsolete (04.2022).
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

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

STREAMABLE(Teammate,
{
  Vector2f getEstimatedPosition(unsigned time) const;

  static Vector2f getEstimatedPosition(const Pose2f& pose, const Vector2f& target, float speed,
                                       const int timeSinceUpdate),

  (int)(-1) number,
  (bool)(false) isGoalkeeper, /**< This is for a teammate what \c theGameState.isGoalkeeper() is for the player itself. */

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

/**
 * @struct TeammateData
 * Collection of teammate information
 */
STREAMABLE(TeamData,
{
  void draw() const,

  (std::vector<Teammate>) teammates,            /**< An unordered(!) list of all teammates that are currently communicating with me */
  (unsigned)(0) receivedMessages,               /**< The number of received (not self) team messages */
  (unsigned)(0) receivedUnsynchronizedMessages, /**< The number of received (not self) team messages that were rejected due to lack of known clock offset */
});
