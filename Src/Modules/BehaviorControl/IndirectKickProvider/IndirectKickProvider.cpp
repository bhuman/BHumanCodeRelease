/**
 * @file IndirectKickProvider.cpp
 *
 * This file implements a module that provides a representation that determines whether a Nao is allowed to shoot a goal.
 *
 * @author Ingo Kelm
 */

#include "IndirectKickProvider.h"
#include "Framework/Settings.h"

MAKE_MODULE(IndirectKickProvider);

void IndirectKickProvider::update(IndirectKick& indirectKick)
{
  // if the GameState switches from one set play to the next indirectKick has to be reset
  if(theGameState.state != previousGameState && theGameState.state != GameState::playing)
    reset(indirectKick);
  previousGameState = theGameState.state;

  // if the GameController is inactive (for a demo), don't use indirect kick
  // or for Scenes with only 1 Player, even if the GameController is active (e.g. BHfast), don't use indirect kick
  if((!theGameState.gameControllerActive || theGameState.ownTeam.numOfActivePlayers() <= 1)
     && !Global::getSettings().scenario.starts_with("SharedAutonomy"))
    indirectKick.allowDirectKick = !disableDirectKicks;
  else
  {
    // Send kick time 0 if goal kicks are currently forbidden in the Shared Autonomy Challenge.
    // If goal kicks are disabled in then challenge (when defending), send the current time instead.
    // This will not enable goal kicks for the teammate, because they are disable there as well, but
    // the tactic will be switch. If not in the Shared Autonomy Challenge and direct kicks are not
    // disabled, the time of the last kick is communicated.
    indirectKick.lastKickTimestamp = theSharedAutonomyRequest.isValid && !theSharedAutonomyRequest.allowGoalKicks ? 0 :
                                     disableDirectKicks ? theFrameInfo.time : theMotionInfo.lastKickTimestamp;

    if(!indirectKick.allowDirectKick || Global::getSettings().scenario.starts_with("SharedAutonomy"))
    {
      updateLastBallContactTimestamps(indirectKick);

      indirectKick.allowDirectKick = false;
      indirectKick.sacAlternateTactic = false;
      for(const unsigned lastBallContactTimestamp : indirectKick.lastBallContactTimestamps)
      {
        // if another team member has played the ball after the last reset, direct kicks are allowed for this NAO
        if(lastBallContactTimestamp > indirectKick.lastSetPlayTime)
        {
          indirectKick.allowDirectKick = !disableDirectKicks;
          indirectKick.sacAlternateTactic = true;
          break;
        }
      }
    }
  }
}

void IndirectKickProvider::reset(IndirectKick& indirectKick)
{
  indirectKick.allowDirectKick = false;
  indirectKick.lastSetPlayTime = theFrameInfo.time;
}

void IndirectKickProvider::updateLastBallContactTimestamps(IndirectKick& indirectKick)
{
  // gets lastKickTimestamps from other TeamMembers, it's sufficient if every player just provides their own lastKickTimestamp (if they send a message to all teammembers after kicking)
  for(const ReceivedTeamMessage& message : theReceivedTeamMessages.messages)
  {
    if(message.number >= Settings::lowestValidPlayerNumber)
      indirectKick.lastBallContactTimestamps[message.number - Settings::lowestValidPlayerNumber] = message.theIndirectKick.lastKickTimestamp;
  }
}
