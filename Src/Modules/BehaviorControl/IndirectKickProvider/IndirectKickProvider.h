/**
 * @file IndirectKickProvider.h
 *
 * This file declares a module that provides a representation that determines whether a Nao is allowed to shoot a goal.
 *
 * @author Ingo Kelm
 */

#pragma once

#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/Communication/GameControllerData.h"
#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RefereeSignal.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Math/RingBuffer.h"

MODULE(IndirectKickProvider,
{,
  USES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameControllerData),
  REQUIRES(GameState),
  REQUIRES(IMUValueState),
  REQUIRES(MotionInfo),
  REQUIRES(ReceivedTeamMessages),
  REQUIRES(RefereeSignal),
  USES(RobotPose),
  REQUIRES(TeamData),
  PROVIDES(IndirectKick),
  LOADS_PARAMETERS(
  {,
    (float) ballHasMovedTolerance, /**< Minimum distance between ball measurements to consider the ball to be moving (in mm) */
    (unsigned) ballSaveInterval, /**< Minimum time between ball measurements that are added to the buffer (in ms) */
    (unsigned) directKickTimeout, /**< Time after a goal kick where a direct kick is allowed (in ms) */
    (float) ballPlacementTolerance, /**< Tolerance of a ball positioned by a referee */
    (float) confidenceIntervalToLegalPositionFree, /**< If the next legal position for the ball is outside this interval the ball is considered free. standard deviation based on the current quality of the ball model */
    (unsigned) freeKickDuration, /**< Minimum time a goal kick needs to take to do a direct kick (in ms) */
    (unsigned) refereePenaltyDelay, /**<  (in ms) */
    (unsigned) minOwnFreeKickDelay, /**< Delay after the start of a free kick until a kick is valid. (in ms) */
  }),
});

class IndirectKickProvider : public IndirectKickProviderBase
{
private:
  GameState::State previousGameState = GameState::playing; /**< game state of the last frame */
  GameState::State previousFreeKick = GameState::playing; /**< last free kick */
  std::array<unsigned, Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1> lastBallContactTimestamps = {}; /**< Array of last deliberate ball contact for each player (index) */
  RingBuffer<Vector2f, 30> ballPositions; /**< Buffer of (robot-relative) ball positions to check whether the ball moved. */
  unsigned lastBallAddedToBuffer = 0; /**< Timestamp of the last ball percept that has been added to the buffer. */
  unsigned timeSinceFreeKickEnded = 0; /**< time since last free kick ended */
  unsigned timeSinceFreeKickStarted = 0; /**< time since last free kick started */
  unsigned lastNumOfActivePlayers = 0; /**<  Number of active players in the last frame */
  bool kickInOpponentTeamDetected = false;
  std::array<GameState::PlayerState, Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1> playerStates = {}; /**< The states of the players, indexed by jersey number - Settings::lowestValidPlayerNumber. */

  /**
   * Updates whether the next kick would be a direct kick or an indirect kick.
   * @param indirectKick The IndirectKick representation.
   */
  void update(IndirectKick& indirectKick) override;

  /**
   * Resets the indirect kick state.
   * @param indirectKick The IndirectKick representation.
   */
  void reset(IndirectKick& indirectKick);

  /**
   * Updates the Timestamps, when a player deliberately played a ball since the last reset (playing phase interruption). Index = player number - Settings::lowestValidPlayerNumber (1).
   */
  void updateLastBallContactTimestamps();

  void updateBallBuffer();

  void updatePlayerStates();

  bool playerIllegalPosition();

  bool checkBallHasMoved() const;

  bool checkForRefereeSignal(const RefereeSignal::Signal signal) const;
};
