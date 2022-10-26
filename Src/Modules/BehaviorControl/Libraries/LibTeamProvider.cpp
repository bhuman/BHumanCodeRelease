/**
 * @file LibTeamProvider.cpp
 */

#include "LibTeamProvider.h"
#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include <cmath>
#include <limits>

MAKE_MODULE(LibTeamProvider, behaviorControl);

void LibTeamProvider::update(LibTeam& libTeam)
{
  libTeam.strikerPlayerNumber = getStrikerPlayerNumber();
  libTeam.iAmClosestToBall = iAmClosestToBall();
  libTeam.numberOfNonKeeperTeammateInOwnGoalArea = numberOfNonKeeperTeammateInOwnGoalArea();
  libTeam.getTeammatePosition = [this](int player) -> Vector2f
  {
    return getTeammatePosition(player);
  };
  libTeam.getBallPosition = [this](int player) -> Vector2f
  {
    return getBallPosition(player);
  };
}

Vector2f LibTeamProvider::getTeammatePosition(int player) const
{
  if(player == theGameState.playerNumber)
    return theRobotPose.translation;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.number == player)
      return teammate.getEstimatedPosition(theFrameInfo.time);
  }
  return Vector2f(1000000.f, 1000000.f);
}

int LibTeamProvider::getStrikerPlayerNumber() const
{
  if(theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall))
    return theGameState.playerNumber;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall))
    {
      return teammate.number;
    }
  }
  return -1;
}

Vector2f LibTeamProvider::getBallPosition(int player) const
{
  if(player == theGameState.playerNumber)
    return theBallModel.estimate.position;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.number == player)
    {
      return teammate.theBallModel.estimate.position;
    }
  }
  return Vector2f::Zero();
}

int LibTeamProvider::numberOfNonKeeperTeammateInOwnGoalArea(const float distanceThreshold) const
{
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.theStrategyStatus.role != PositionRole::toRole(PositionRole::goalkeeper)
       && theLibPosition.isNearOwnGoalArea(teammate.getEstimatedPosition(theFrameInfo.time), distanceThreshold, distanceThreshold))
      return teammate.number;
  }
  return -1;
}

bool LibTeamProvider::iAmClosestToBall() const
{
  const Vector2f ballPositionOnField(theFieldBall.recentBallPositionOnField(3000));
  const float distanceToBall = (theRobotPose.translation - ballPositionOnField).squaredNorm();
  for(const Teammate& teammate : theTeamData.teammates)
  {
    if((teammate.getEstimatedPosition(theFrameInfo.time) - ballPositionOnField).squaredNorm() < distanceToBall)
      return false;
  }
  return true;
}
