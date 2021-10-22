/**
 * @file ExtendedGameInfoProvider.h
 *
 * This file declares a module that provides extended information about the game state.
 *
 * @author Andreas Stolpmann
 * @author Tim Laue
 * @author Jesse Richter-Klug
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"

MODULE(ExtendedGameInfoProvider,
{,
  USES(BehaviorStatus),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(RobotInfo),
  PROVIDES(ExtendedGameInfo),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(15000) timeBetweenWhistleAndGCPlaying, /**< The delay between the whistle and the playing signal from the GameController. [ms] */
    (unsigned)(600000) fullHalfDuration, /**< The duration of one half of the game [ms]. */
    (int)(23000) minPenaltyTime, /**< The minimum time a robot must be penalized to actually believe it [ms]. */
    (int)(8000) minPenaltyTimeIP, /**< The minimum time a robot must be penalized for Illegal Position in Set to actually believe it [ms]. */
  }),
});

class ExtendedGameInfoProvider : public ExtendedGameInfoProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param extendedGameInfo The representation updated.
   */
  void update(ExtendedGameInfo& extendedGameInfo) override;

  decltype(RoboCup::RoboCupGameControlData::state) rawGameStateLastFrame = STATE_INITIAL;          /**< The raw game state that was active in the last frame. */
  decltype(RoboCup::RoboCupGameControlData::state) gameStateLastFrame = STATE_INITIAL;             /**< The game state that was active in the last frame. */
  decltype(RoboCup::RoboCupGameControlData::gamePhase) gamePhaseLastFrame = GAME_PHASE_NORMAL;     /**< The game phase that was active in the last frame. */
  decltype(RoboCup::RoboCupGameControlData::setPlay) setPlayLastFrame = SET_PLAY_NONE;             /**< The set play that was active in the last frame. */
  decltype(RoboCup::RobotInfo::penalty) penaltyLastFrame = PENALTY_NONE;                           /**< The penalty that this robot had int the last frame. */
  decltype(RoboCup::RoboCupGameControlData::state) gameStateBeforeCurrent = STATE_INITIAL;         /**< The game state that was active prior to the current game state. */
  decltype(RoboCup::RoboCupGameControlData::gamePhase) gamePhaseBeforeCurrent = GAME_PHASE_NORMAL; /**< The game phase that was active prior to the current game phase. */
  RobotInfo::Mode modeLastFrame = RobotInfo::Mode::unstiff;                                        /**< The mode that this robot had in the last frame. */

  unsigned timeWhenLastReadyStarted = 0;    /**< The name says it all. */
  unsigned timeWhenLastSetStarted = 0;      /**< The name says it all. */
  unsigned timeWhenLastPlayingStarted = 0;  /**< The name says it all. */
  unsigned timeWhenLastPenaltyEnded = 0;    /**< The name says it all. */
  unsigned timeWhenLastFreeKickStarted = 0; /**< The name says it all. */
  unsigned timeWhenLastFreeKickEnded = 0;   /**< The name says it all. */
  unsigned timeWhenLastCalibrationStarted = 0; /**< The name says it all. */

  bool manuallyPlaced = false;               /**< Was the robot manually placed in the set state? */
  unsigned timeWhenPenalized = 0;            /**< When was the robot penalized. */
  bool receivedGameControllerPacket = false; /**< Was a GameController packet already received? */
};
