/**
 * @file LibTeamProvider.cpp
 */

#include <cmath>
#include <limits>
#include "LibTeamProvider.h"

MAKE_MODULE(LibTeamProvider, behaviorControl);

void LibTeamProvider::update(LibTeam& libTeam)
{
  libTeam.keeperPlayerNumber = getKeeperPlayerNumber();
  libTeam.strikerPlayerNumber = getStrikerPlayerNumber();
  libTeam.keeperPose = getKeeperPose();
  libTeam.strikerPose = getStrikerPose();
  libTeam.iAmClosestToBall = iAmClosestToBall();
  libTeam.minTeammateDistanceToBall = getMinTeammateDistanceToBall();
  libTeam.numberOfNonKeeperTeammateInOwnGoalArea = numberOfNonKeeperTeammateInOwnGoalArea();
  libTeam.numberOfBallPlayingTeammate = getStrikerPlayerNumber();
  libTeam.getTeammatePose = [this](int player)->Pose2f
  {
    return getTeammatePose(player);
  };
  libTeam.getActivity = [this](int player)->BehaviorStatus::Activity
  {
    return getActivity(player);
  };
  libTeam.getStatus = [this](int player)->Teammate::Status
  {
    return getStatus(player);
  };
  libTeam.getBallPosition = [this](int player)->Vector2f
  {
    return getBallPosition(player);
  };
  libTeam.getTimeToReachBall = [this](int player)->const TimeToReachBall*
  {
    return getTimeToReachBall(player);
  };
}

Pose2f LibTeamProvider::getKeeperPose() const
{
  if(theTeamBehaviorStatus.role.isGoalkeeper())
    return theRobotPose;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED && teammate.theTeamBehaviorStatus.role.isGoalkeeper())
    {
      return teammate.theRobotPose;
    }
  }
  return Pose2f(0.f, 1000000.f, 1000000.f);
}

Pose2f LibTeamProvider::getStrikerPose() const
{
  if(theTeamBehaviorStatus.role.playsTheBall())
    return theRobotPose;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED && teammate.theTeamBehaviorStatus.role.playsTheBall())
    {
      return teammate.theRobotPose;
    }
  }
  return Pose2f(0.f, 1000000.f, 1000000.f);
}

Pose2f LibTeamProvider::getTeammatePose(int player) const
{
  if(player == theRobotInfo.number)
    return theRobotPose;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.number == player)
    {
      if(teammate.status != Teammate::PENALIZED)
        return teammate.theRobotPose;
      else
        break;
    }
  }
  return Pose2f(0.f, 1000000.f, 1000000.f);
}

int LibTeamProvider::getKeeperPlayerNumber() const
{
  if(theTeamBehaviorStatus.role.isGoalkeeper())
    return theRobotInfo.number;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED && teammate.theTeamBehaviorStatus.role.isGoalkeeper())
    {
      return teammate.number;
    }
  }
  return -1;
}

int LibTeamProvider::getStrikerPlayerNumber() const
{
  if(theTeamBehaviorStatus.role.playsTheBall())
    return theRobotInfo.number;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED && teammate.theTeamBehaviorStatus.role.playsTheBall())
    {
      return teammate.number;
    }
  }
  return -1;
}

BehaviorStatus::Activity LibTeamProvider::getActivity(int player) const
{
  if(player == theRobotInfo.number)
    return theBehaviorStatus.activity;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.number == player)
    {
      if(teammate.status != Teammate::PENALIZED)
        return teammate.theBehaviorStatus.activity;
      else
        break;
    }
  }
  return BehaviorStatus::unknown;
}

Teammate::Status LibTeamProvider::getStatus(int player) const
{
  if(player == theRobotInfo.number)
    return Teammate::PLAYING;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.number == player)
    {
      return teammate.status;
    }
  }
  return Teammate::PENALIZED;
}

Vector2f LibTeamProvider::getBallPosition(int player) const
{
  if(player == theRobotInfo.number)
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

const TimeToReachBall* LibTeamProvider::getTimeToReachBall(int player) const
{
  if(player == theRobotInfo.number)
    return &(theTeamBehaviorStatus.timeToReachBall);
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.number == player)
      return &(teammate.theTeamBehaviorStatus.timeToReachBall);
  }
  return nullptr;
}

int LibTeamProvider::numberOfNonKeeperTeammateInOwnGoalArea(const float distanceThreshold) const
{
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED
       && !teammate.theTeamBehaviorStatus.role.isGoalkeeper()
       && theLibPosition.isNearOwnGoalArea(teammate.theRobotPose.translation, distanceThreshold, distanceThreshold))
    {
      return teammate.number;
    }
  }
  return -1;
}

bool LibTeamProvider::iAmClosestToBall() const
{
  const Vector2f ballPositionOnField(theFieldBall.recentBallPositionOnField(3000));
  const float distanceToBall = (theRobotPose.translation - ballPositionOnField).squaredNorm();
  for(const Teammate& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED && (teammate.theRobotPose.translation - ballPositionOnField).squaredNorm() < distanceToBall)
      return false;
  }
  return true;
}

float LibTeamProvider::getMinTeammateDistanceToBall() const
{
  const Vector2f ballPositionOnField(theFieldBall.recentBallPositionOnField(3000));
  float minDistance = std::numeric_limits<float>::max();
  for(const Teammate& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PENALIZED)
    {
      const float distance = (teammate.theRobotPose.translation - ballPositionOnField).squaredNorm();
      if(distance < minDistance)
        minDistance = distance;
    }
  }
  return std::sqrt(minDistance);
}
