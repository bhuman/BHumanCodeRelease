/**
 * @file ExtendedGameInfo.h
 *
 * This file defines a representation that contains more (especially temporal) information
 * about the game state than is contained in the data from the GameController.
 *
 * @author Andreas Stolpmann
 * @author Tim Laue
 * @author Jesse Richter-Klug
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Communication/RoboCupGameControlData.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ExtendedGameInfo,
{,
  (unsigned)(0) timeWhenLastSearchedBall,    /**< Timestamp when the robot last searched the ball. */
  (int)(0) timeSinceLastPenaltyEnded,        /**< Time since the last penalty ended (or 0 if currently penalized) [ms]. */
  (unsigned)(0) timeWhenLastReadyStarted,    /**< Timestamp when the last ready state started. */
  (int)(0) timeSinceReadyStarted,            /**< Time since the current ready state started (or 0 if not ready) [ms]. */
  (unsigned)(0) timeWhenLastSetStarted,      /**< Timestamp when the last set state started. */
  (int)(0) timeSinceSetStarted,              /**< Time since the current set state started (or 0 if not set) [ms]. */
  (unsigned)(0) timeWhenLastPlayingStarted,  /**< Timestamp when the last playing state started. */
  (int)(0) timeSincePlayingStarted,          /**< Time since the current playing state started (or 0 if not playing) [ms]. */
  (unsigned)(0) timeWhenLastFreeKickStarted, /**< Timestamp when the last free kick started. */
  (int)(0) timeSinceFreeKickStarted,         /**< Time since the last free kick started [ms]. */
  (unsigned)(0) timeWhenLastFreeKickEnded,   /**< Timestamp when the last free kick ended. */
  (int)(0) timeSinceLastFreeKickEnded,       /**< Time since the last free kick ended [ms]. */
  (unsigned)(0) timeWhenLastCalibrationStarted, /**< Timestamp when robot last switched to Mode::calibration. */
  (int)(0) timeSinceLastCalibrationStarted,  /**< Time since the calibration started [ms]. */

  (decltype(RoboCup::RoboCupGameControlData::state))(STATE_INITIAL) gameStateLastFrame,             /**< The game state in the last frame. */
  (decltype(RoboCup::RoboCupGameControlData::gamePhase))(GAME_PHASE_NORMAL) gamePhaseLastFrame,     /**< The game phase in the last frame. */
  (decltype(RoboCup::RoboCupGameControlData::setPlay))(SET_PLAY_NONE) setPlayLastFrame,             /**< The set play in the last frame. */
  (decltype(RoboCup::RobotInfo::penalty))(PENALTY_NONE) penaltyLastFrame,                           /**< The penalty in the last frame. */
  (decltype(RoboCup::RoboCupGameControlData::state))(STATE_INITIAL) gameStateBeforeCurrent,         /**< The game state before the current game state. */
  (decltype(RoboCup::RoboCupGameControlData::gamePhase))(GAME_PHASE_NORMAL) gamePhaseBeforeCurrent, /**< The game phase before the current game phase. */

  (bool)(false) didNotHearWhistleThisTime,       /**< Whether the state is playing and the transition to playing was not initiated by a whistle. */
  (bool)(false) walkingInFromSidelines,          /**< Whether the state is ready and the robots started from the sidelines. */
  (bool)(false) startingCalibration,             /**< The calibration mode just engaged. */
  (bool)(false) returnFromGameControllerPenalty, /**< The robot was unpenalized by the GameController and believes it. */
  (bool)(false) returnFromManualPenalty,         /**< The robot was unpenalized by the GameController and believes it. */
  (bool)(false) manuallyPlaced,                  /**< The robot has been set to its kick-off position during SET by the referee */
});
