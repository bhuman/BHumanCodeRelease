/**
 * @file ExtendedGameStateProvider.h
 *
 * This file implements a module that provided information that is derived from the game state
 * but not suitable for being exchanged between threads.
 *
 * @author Arne Hasselbring
 */

#include "ExtendedGameStateProvider.h"

MAKE_MODULE(ExtendedGameStateProvider);

void ExtendedGameStateProvider::update(ExtendedGameState& extendedGameState)
{
  extendedGameState.stateLastFrame = stateLastFrame;
  extendedGameState.playerStateLastFrame = playerStateLastFrame;
  extendedGameState.messageBudgetLastFrame = messageBudgetLastFrame;

  extendedGameState.timeWhenStateStarted[theGameState.state] = theGameState.timeWhenStateStarted;
  extendedGameState.timeWhenPlayerStateStarted[theGameState.playerState] = theGameState.timeWhenPlayerStateStarted;

  if(theGameState.isPenalized() && !extendedGameState.wasPenalized())
    timeWhenPenalized = theGameState.timeWhenPlayerStateStarted; // Don't use theFrameInfo.time because the timestamp may have been backdated.

  if(!theGroundContactState.contact)
    lastTimeWithoutGroundContact = theFrameInfo.time;

  extendedGameState.returnFromGameControllerPenalty = false;
  extendedGameState.returnFromManualPenalty = false;
  if(!theGameState.isPenaltyShootout() && !theGameState.isPenalized())
  {
    // Return from manual penalty is directly set without any further checks,
    // as no mistakes happen accidentally (in general).
    if(playerStateLastFrame == GameState::penalizedManual)
    {
      extendedGameState.returnFromManualPenalty = true;
    }
    // A "normal" return from penalty requires some additional checks:
    else if(GameState::isPenalized(playerStateLastFrame) &&
            playerStateLastFrame != GameState::penalizedIllegalMotionInSet)
    {
      // 1. The penalty must have lasted a minimum period of time. The GameController operator might have
      //    clicked the wrong robot (or made a similar mistake) and reverted the penalty. Usually, this happens
      //    after a few seconds. If more time has passed (as configured by the two parameters), we consider this as
      //    a normal return from penalty, i.e. the robot is placed at the sideline.
      if(theFrameInfo.getTimeSince(timeWhenPenalized) > (playerStateLastFrame == GameState::penalizedIllegalPositionInSet ? minPenaltyTimeIPS : minPenaltyTime))
        extendedGameState.returnFromGameControllerPenalty = true;
      // 2. The penalty is officially aborted by the referee. Thus, it ends earlier but we still assume a minimum period of time
      //    for having been penalized. Furthermore, we check if the robot was move during the penalty, i.e. probably positioned at the sideline.
      else if(theFrameInfo.getTimeSince(timeWhenPenalized) > minPenaltyTimeAbortedPenalty && timeWhenPenalized < lastTimeWithoutGroundContact)
        extendedGameState.returnFromGameControllerPenalty = true;
    }
  }

  stateLastFrame = theGameState.state;
  playerStateLastFrame = theGameState.playerState;
  messageBudgetLastFrame = theGameState.ownTeam.messageBudget;
}
