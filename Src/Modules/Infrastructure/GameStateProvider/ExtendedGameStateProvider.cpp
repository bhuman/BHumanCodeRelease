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

  extendedGameState.returnFromGameControllerPenalty = false;
  extendedGameState.returnFromManualPenalty = false;
  if(!theGameState.isPenaltyShootout() && !theGameState.isPenalized())
  {
    if(playerStateLastFrame == GameState::penalizedManual)
      extendedGameState.returnFromManualPenalty = true;
    else if(GameState::isPenalized(playerStateLastFrame) &&
            playerStateLastFrame != GameState::penalizedIllegalMotionInSet &&
            playerStateLastFrame != GameState::penalizedIllegalMotionInStandby &&
            theFrameInfo.getTimeSince(timeWhenPenalized) > (playerStateLastFrame == GameState::penalizedIllegalPositionInSet ? minPenaltyTimeIPS : minPenaltyTime))
      extendedGameState.returnFromGameControllerPenalty = true;
  }

  stateLastFrame = theGameState.state;
  playerStateLastFrame = theGameState.playerState;
  messageBudgetLastFrame = theGameState.ownTeam.messageBudget;
}
