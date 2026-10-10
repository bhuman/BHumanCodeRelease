/**
 * @file IndirectKickProvider.cpp
 *
 * TODO
 *
 * @author Philip Reichenberg
 */

#include "IndirectKickProvider.h"
#include "Framework/Settings.h"

MAKE_MODULE(IndirectKickProvider);

void IndirectKickProvider::update(IndirectKick& indirectKick)
{
  indirectKick.oldAllowDirectKick = indirectKick.allowDirectKick;

  // During stop we do nothing. Wait until play is resumed, then we will also call the reset
  if(theGameState.stopped)
    return;

  // If the GameState switches from one set play to the next, indirectKick has to be reset
  // Same if the player is penalized for illegal position after a set play

  if(theGameState.state != previousGameState && theGameState.isForOwnTeam()
     && (theGameState.isKickOff() || theGameState.isKickIn() || theGameState.isIndirectFreeKick()))
    reset(indirectKick);

  indirectKick.lastKickTimestamp = theMotionInfo.lastKickTimestamp;

  previousGameState = theGameState.state;

  // When we have 2 or less players, we do not need to wait for another player to touch the ball
  if(theGameState.ownTeam.numOfActivePlayers() <= 2)
  {
    indirectKick.allowDirectKick |= !theGameState.isFreeKick() || theGameState.isForOpponentTeam();
    indirectKick.lastStriker = 0;
    return;
  }

  if(!indirectKick.allowDirectKick)
  {
    if(indirectKick.lastSetPlayTime < theMotionInfo.lastKickTimestamp)
    {
      indirectKick.allowDirectKick = true;
    }
  }

  int strikerCount = 0;
  indirectKick.lastStriker = 0;
  if(indirectKick.lastKickTimestamp > indirectKick.lastSetPlayTime)
  {
    strikerCount++;
    if(theFrameInfo.getTimeSince(indirectKick.lastKickTimestamp) < strikerTimeout)
      indirectKick.lastStriker = theGameState.playerNumber;
  }
  for(const Teammate& teammate : theTeamData.teammates)
  {
    if(teammate.theIndirectKick.lastKickTimestamp > indirectKick.lastSetPlayTime)
    {
      indirectKick.allowDirectKick = true;
      strikerCount++;
      if(theFrameInfo.getTimeSince(teammate.theIndirectKick.lastKickTimestamp) < strikerTimeout)
        indirectKick.lastStriker = teammate.number;
    }
  }

  if(strikerCount > 1)
  {
    indirectKick.lastStriker = 0;
  }
}

void IndirectKickProvider::reset(IndirectKick& indirectKick)
{
  indirectKick.allowDirectKick = false;
  indirectKick.lastSetPlayTime = theFrameInfo.time;
}
