/**
 * @file TeamDataProvider.cpp
 *
 * This file implements a module that converts \c ReceivedTeamMessages to \c TeamData.
 * In the middle term, this module and \c TeamData should disappear.
 *
 * @author Arne Hasselbring
 */

#include "TeamDataProvider.h"

MAKE_MODULE(TeamDataProvider);

void TeamDataProvider::update(TeamData& teamData)
{
  teamData.receivedMessages += static_cast<unsigned int>(theReceivedTeamMessages.messages.size());
  teamData.receivedUnsynchronizedMessages += theReceivedTeamMessages.unsynchronizedMessages;
  for(const ReceivedTeamMessage& teamMessage : theReceivedTeamMessages.messages)
    handleMessage(getTeammate(teamData, teamMessage.number), teamMessage);

  for(auto it = teamData.teammates.begin(); it != teamData.teammates.end();)
  {
    if(theGameState.ownTeam.playerStates[it->number - Settings::lowestValidPlayerNumber] != GameState::active &&
       !theLibDemo.isOneVsOneDemoActive)
      it = teamData.teammates.erase(it);
    else
    {
      it->isGoalkeeper = theGameState.ownTeam.isGoalkeeper(it->number);
      ++it;
    }
  }
}

Teammate& TeamDataProvider::getTeammate(TeamData& teamData, int number) const
{
  for(Teammate& teammate : teamData.teammates)
    if(teammate.number == number)
      return teammate;
  return teamData.teammates.emplace_back();
}

void TeamDataProvider::handleMessage(Teammate& teammate, const ReceivedTeamMessage& teamMessage) const
{
  teammate.number = teamMessage.number;
  teammate.theRobotStatus = teamMessage.theRobotStatus;
  teammate.theRobotPose = teamMessage.theRobotPose;
  teammate.theBallModel = teamMessage.theBallModel;
  teammate.theFrameInfo = teamMessage.theFrameInfo;
  teammate.theBehaviorStatus = teamMessage.theBehaviorStatus;
  teammate.theWhistle = teamMessage.theWhistle;
  teammate.theStrategyStatus = teamMessage.theStrategyStatus;
  teammate.theIndirectKick = teamMessage.theIndirectKick;
  teammate.theInitialToReady = teamMessage.theInitialToReady;
}
