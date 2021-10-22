/**
 * @file WhistleHandler.h
 * Modifies the GameInfo to factor in if a whistle was
 * heard by the teammates to set the state to playing.
 * @author Andreas Stolpmann
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallInGoal.h"

MODULE(WhistleHandler,
{,
  USES(FieldBall),
  REQUIRES(BallInGoal),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(TeamData),
  REQUIRES(Whistle),
  PROVIDES(GameInfo),
  LOADS_PARAMETERS(
  {,
    (int) maxTimeDifference, /**< Time window size of all whistles considered together in ms. */
    (float) minAvgConfidence, /**< Minimum confidence of all considered whistles required for a detection. */
    (bool) useWhistleAfterGoal, /**< Whether the whistle can switch the state to ready. */
    (int) ignoreWhistleAfterKickOff, /**< Time whistles are ignored after switching to playing after a kick-off (in ms). */
    (int) ignoreWhistleAfterPenaltyKick, /**< Time whistles are ignored after switching to playing after the begin of a penalty kick (in ms). */
    (bool) checkBallForGoal, /**< Consider ball position when checking whether a goal was announced. */
    (int) acceptBallInGoalDelay, /**< Time in ms since when a ball has to have been seen in a goal, to accept as a valid goal when referee whistles */
    (int) gameControllerDelay, /**< Delay after which the GameController should send a changed game state (in ms).*/
    (int) gameControllerOperatorDelay, /**< Delay with which the operator of the GameController enters the referee's decisions. */
    (int) acceptPastWhistleDelay, /**< How old can whistles be to be accepted after canceling READY (in ms)? */
  }),
});

class WhistleHandler : public WhistleHandlerBase
{
private:
  unsigned timeOfLastStateChange = 0; /**< The time when the effective game state last changed. */
  uint8_t lastGameState = STATE_INITIAL; /**< The previous raw game state. */
  uint8_t guessedGameState = STATE_INITIAL; /**< The guessed game state. Use if different from STATE_INITIAL. */
  uint8_t kickingTeam = 0; /**<  The guessed number of the team that has kick-off. */
  std::vector<unsigned> penaltyTimes; /**< The times when players last were last penalized for Illegal Motion in Set. */

  /**
   * Check whether a whistle was heard. It is checked whether the average confidence of all
   * players that actually listened and heard a whistle in a certain time window is high enough.
   * @return Did the team hear a whistle?
   */
  bool checkForWhistle() const;

  /**
   * Check whether at least one player received a Illegal-Motion-in-Set-penalty since the last
   * change of the game state.
   */
  bool checkForIllegalMotionPenalty();

  /**
   * Check whether the ball was in the last 5 seconds considered as in goal
   * @return Was the ball in the last 5 seconds in a goal?
   */
  bool checkForBallPosition();

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theGameInfo The representation updated.
   */
  void update(GameInfo& theGameInfo) override;
};
