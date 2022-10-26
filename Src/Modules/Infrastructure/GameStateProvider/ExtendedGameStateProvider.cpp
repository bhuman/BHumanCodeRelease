/**
 * @file ExtendedGameStateProvider.h
 *
 * This file implements a module that provided information that is derived from the game state
 * but not suitable for being exchanged between threads.
 *
 * @author Arne Hasselbring
 */

#include "ExtendedGameStateProvider.h"

MAKE_MODULE(ExtendedGameStateProvider, infrastructure);

void ExtendedGameStateProvider::update(ExtendedGameState& extendedGameState)
{
  extendedGameState.stateLastFrame = stateLastFrame;
  extendedGameState.playerStateLastFrame = playerStateLastFrame;
  extendedGameState.messageBudgetLastFrame = messageBudgetLastFrame;

  extendedGameState.timeWhenStateStarted[theGameState.state] = theGameState.timeWhenStateStarted;
  extendedGameState.timeWhenPlayerStateStarted[theGameState.playerState] = theGameState.timeWhenPlayerStateStarted;

  // The following does not work for players that are penalized for illegal motion in set before we request manual placement since we can't distinguish this from the robot being reset to its original position before it moved. Given that manual placement was removed from the rules, this code will likely be removed soon anyway.
  if((theGameState.state != GameState::waitForOwnKickOff && theGameState.state != GameState::waitForOpponentKickOff) ||
     (theGameState.playerState != GameState::active && theGameState.playerState != GameState::penalizedIllegalMotionInSet))
    extendedGameState.manuallyPlaced = false;
  else if((theGameState.playerState == GameState::active && theFrameInfo.getTimeSince(extendedGameState.timeWhenPlayerStateStarted[GameState::active]) > 5000) &&
           theFallDownState.state == FallDownState::pickedUp)
    extendedGameState.manuallyPlaced = true;

  if(theGameState.isPenalized() && !extendedGameState.wasPenalized())
    timeWhenPenalized = theGameState.timeWhenPlayerStateStarted; // Don't use theFrameInfo.time because the timestamp may have been backdated.

  extendedGameState.returnFromGameControllerPenalty = false;
  extendedGameState.returnFromManualPenalty = false;
  if(!theGameState.isPenaltyShootout() && !theGameState.isPenalized())
  {
    if(playerStateLastFrame == GameState::penalizedManual)
      extendedGameState.returnFromManualPenalty = true;
    else if(GameState::isPenalized(playerStateLastFrame) && playerStateLastFrame != GameState::penalizedIllegalMotionInSet &&
            theFrameInfo.getTimeSince(timeWhenPenalized) > (playerStateLastFrame == GameState::penalizedIllegalPositionInSet ? minPenaltyTimeIPS : minPenaltyTime))
      extendedGameState.returnFromGameControllerPenalty = true;
  }

  stateLastFrame = theGameState.state;
  playerStateLastFrame = theGameState.playerState;
  messageBudgetLastFrame = theGameState.ownTeam.messageBudget;
}
