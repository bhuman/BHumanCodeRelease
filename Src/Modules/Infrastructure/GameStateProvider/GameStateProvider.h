/**
 * @file GameStateProvider.h
 *
 * This file declares a module that provides a condensed representation of the game state.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Communication/GameControllerData.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/DampingState.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Framework/Module.h"
#include "Math/RingBuffer.h"

MODULE(GameStateProvider,
{,
  USES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(DampingState),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameControllerData),
  REQUIRES(IMUValueState),
  REQUIRES(MotionInfo),
  USES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(Whistle),
  PROVIDES(GameState),
  LOADS_PARAMETERS(
  {,
    (int) gameControllerTimeout, /**< Time after which the GameController is assumed to be inactive (in ms). */
    (int) ballSaveInterval, /**< Minimum time between ball measurements that are added to the buffer (in ms). */
    (float) ballHasMovedTolerance, /**< Minimum distance between ball measurements to consider the ball to be moving (in mm). */
    (float) ballHasMovedDuringKickOffTolerance, /**< Minimum distance between ball measurements to consider the ball to be moving during kick-off (in mm). */
    (float) ballHasMovedCloseToRobotThreshold, /**< Offset from the minimum distance that a ball can have to a legally positioned robot before it is considered as having moved (in mm). */
    (unsigned) ballOutOfCenterCircleCounterThreshold, /**< How often the ball must have been "seen" out of the center circle to leave the kick-off state. */
    (float) ballOutOfCenterCircleTolerance, /**< The center circle is extended by this offset (in mm). */
    (int) maxWhistleTimeDifference, /**< Time window size of all whistles considered together (in ms). */
    (float) minWhistleAverageConfidence, /**< Minimum confidence of all considered whistles required for a detection. */
    (bool) useMaxWhistleConfidence, /**< Use the maximum whistle confidence instead of the average. */
    (int) gameControllerOperatorDelay, /**< Delay with which the operator of the GameController enters the referee's decisions. */
    (float) confidenceIntervalToLegalPositionFree, /**< If the next legal position for the ball is outside this interval the ball is considered free. standard deviation based on the current quality of the ball model. */
    (float) confidenceIntervalToLegalPositionNotFree, /**< If the next legal position for the ball is outside this interval the ball is not considered free anymore. standard deviation based on the current quality of the ball model. */
    (float) ballPlacementTolerance, /**< Tolerance of a ball positioned by a referee */
    (unsigned) minOwnFreeKickDelay, /**< Delay after the start of a free kick until a kick is valid. (in ms) */

    /* The following are constants for robot state switches. */
    (unsigned) pressDurationUnstiffToPrepare, /**< Time the stand button needs to be pressed to switch from inactive to active. */
    (unsigned) ignoreMainButtonPressDuration, /**< If button is pressed for too long, do not consider it pressed. */

    /* The following are constants from the rule book: */
    (int) kickOffSetupDuration, /**< Duration of a READY phase for a kick-off (in ms). */
    (int) kickOffDuration, /**< Duration of the kick-off before the ball is free (in ms). */
    (int) penaltyKickSetupDuration, /**< Duration of a READY phase for a penalty kick (in ms). */
    (int) penaltyKickDuration, /**< Duration of a penalty kick (both during the game and in a penalty shoot-out) (in ms). */
    (int) freeKickDuration, /**< Duration of a free kick (in ms). */
    (int) playingSignalDelay, /**< Delay after which the GameController sends the PLAYING state after a kick-off / penalty kick (in ms). */
  }),
});

class GameStateProvider : public GameStateProviderBase
{
  /**
   * Updates the game state.
   * @param gameState The updated representation.
   */
  void update(GameState& gameState) override;

  /**
   * Resets the game state and members.
   * @param gameState The reset representation.
   */
  void reset(GameState& gameState);

  /**
   * Checks whether a whistle was heard. It is checked whether the average confidence of all
   * players that actually listened and heard a whistle in a certain time window is high enough.
   * @param from Lower timestamp bound to accept whistles.
   * @param useGameControllerData Look at GameController data to determine which
   *                              teammates are available.
   * @return Did the team hear a whistle?
   */
  bool checkForWhistle(unsigned from, bool useGameControllerData) const;

  /**
   * Checks whether the ball has moved during the current state.
   * @param gameState The current game state.
   * @return Has the ball been moved / touched after the last game state change?
   */
  bool checkBallHasMoved(const GameState& gameState) const;

  /**
   * Checks whether a player as been penalized for motion in SET.
   * @param from Lower timestamp bound on the start of penalties.
   * @return Has a player been penalized for motion in SET after the last (guessed) game state change?
   */
  bool checkForMotionInSetPenalty(unsigned from) const;

  /**
   * Checks whether the ball is on a position that is considered legal in the given state
   * The cases for kickOffs are not (yet) considered as they are handled by "checkBallHasMoved"
   * @param gameState The gameState for which to check
   * @param confidenceIntervalToLegalPosition The confidence interval inside which the ball is still considered legal
   * @return true if the ball is at a plausible position for the given state, false otherwise
   * @author Yannik Meinken
   */
  bool checkBallLegal(const GameState::State& gameState, const float confidenceIntervalToLegalPosition) const;

  /**
   * Updates the buffer of balls for determining whether the ball has moved.
   * @param gameState The current game state.
   */
  void updateBallBuffer(const GameState& gameState);

  /** Updates timestamps when players have been penalized for motion in SET. */
  void updateMotionInSetTimestamps();

  /** Updates timestamps when a player have been penalized for illegal positioning. */
  void updateIllegalPositioningTimestamps();

  /**
   * Extracts the B-Human game phase from raw GameController data.
   * @param gameControllerData The data from the GameController.
   * @return The corresponding B-Human game phase.
   */
  static GameState::Phase convertGameControllerDataToPhase(const GameControllerData& gameControllerData);

  /**
   * Extracts the B-Human game state from raw GameController data.
   * @param gameControllerData The data from the GameController.
   * @return The corresponding B-Human game state.
   */
  GameState::State convertGameControllerDataToState(const GameControllerData& gameControllerData);

  /**
   * Maps GameController penalty macro constants to B-Human enum constants.
   * @param penalty A GameController penalty.
   * @return The corresponding B-Human penalty.
   */
  static GameState::PlayerState convertPenaltyToPlayerState(decltype(RoboCup::RobotInfo::penalty) penalty);

  GameState::State lastGameControllerState = GameState::beforeHalf; /**< The last state sent by the GameController. */
  bool gameStateOverridden = false; /**< Whether the game state is currently overridden (i.e. not the state received by the GameController). */
  unsigned timeWhenStateNotAfterHalf = 0; /**< Last timestamp when the state was not "after half". */
  RingBuffer<Vector2f, 30> ballPositions; /**< Buffer of (robot-relative) ball positions to check whether the ball moved. */
  unsigned lastBallAddedToBuffer = 0; /**< Timestamp of the last ball percept that has been added to the buffer. */
  unsigned ballOutOfCenterCircleCounter = 0; /**< How often the ball was "seen" out of the center circle during the current kick-off. */
  unsigned lastBallUsedForCenterCircleCounter = 0; /**< Timestamp of the last ball percept (or teammates ball) that has been used for the center circle counter. */
  std::vector<unsigned> motionInSetTimestamps; /**< Timestamps for all players when their illegal motion in SET penalty began. */
  std::vector<unsigned> illegalPositioningTimestampsOwnTeam; /**< Timestamps for all players in the own team when their illegal positioning penalty began. */
  std::vector<unsigned> illegalPositioningTimestampsOpponentTeam; /**< Timestamps for all opponent players when their illegal positioning penalty began. */
  unsigned timeOfLastIntegratedGameControllerData = 0; /**< Timestamp of the last GameControllerData that has been integrated into the state. */
  unsigned timeWhenStateStartedBeforeWhistle = 0; /**< Timestamp of the start of the state before a transition was triggered by a whistle. */
  unsigned timeWhenStateStartedBeforeBallMoved = 0; /**< Timestamp of the start of the state before a transition was triggered by ball movement. */
  unsigned timeWhenSwitchedToPlayingViaWhistle = 0; /**< Timestamp when a transition to PLAYING (kick-off/penalty kick/penalty shot) was triggered by a whistle. */
  unsigned minWhistleTimestamp = 0; /**< Lower bound for whistle timestamps to consider for transitions. */
};
