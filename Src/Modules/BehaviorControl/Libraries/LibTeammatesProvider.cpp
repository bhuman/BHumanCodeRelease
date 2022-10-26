/**
 * @file LibTeamProvider.cpp
 * @author Lukas Malte Monerjahn
 */

#include "LibTeammatesProvider.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"

MAKE_MODULE(LibTeammatesProvider, behaviorControl);

void LibTeammatesProvider::update(LibTeammates& libTeammates)
{
  libTeammates.nonKeeperTeammatesInOwnPenaltyArea = nonKeeperTeammatesInOwnPenaltyArea();
  libTeammates.teammatesInOpponentPenaltyArea = teammatesInOpponentPenaltyArea();
}

int LibTeammatesProvider::nonKeeperTeammatesInOwnPenaltyArea() const
{
  float distanceThreshold = outsideDistanceThreshold;
  bool isNear = false;
  if(theLibPosition.isNearOwnPenaltyArea(theRobotPose.translation, -theRobotPose.getXAxisStandardDeviation(), -theRobotPose.getYAxisStandardDeviation()))
    distanceThreshold = insideDistanceThreshold;
  else if(theLibPosition.isNearOwnPenaltyArea(theRobotPose.translation, outsideDistanceThreshold, outsideDistanceThreshold))
    isNear = true;
  int teammatesInPenaltyArea = 0;
  for(auto const& teammate : theTeamData.teammates)
  {
    const Vector2f teammatePosition = teammate.getEstimatedPosition(theFrameInfo.time);
    if(!teammate.isGoalkeeper
       && theLibPosition.isNearOwnPenaltyArea(teammatePosition, distanceThreshold, distanceThreshold))
    {
      if(isNear && !theLibPosition.isNearOwnPenaltyArea(teammatePosition, insideDistanceThreshold, insideDistanceThreshold))
      {
        /// breaks ties between two robots attempting to enter simultaneously
        if(teammate.theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall))
          ++teammatesInPenaltyArea;
      }
      else
        ++teammatesInPenaltyArea;
    }
  }
  return teammatesInPenaltyArea;
}

int LibTeammatesProvider::teammatesInOpponentPenaltyArea() const
{
  float distanceThreshold = outsideDistanceThreshold;
  if(theLibPosition.isNearOpponentPenaltyArea(theRobotPose.translation, -theRobotPose.getXAxisStandardDeviation(), -theRobotPose.getYAxisStandardDeviation()))
    distanceThreshold = insideDistanceThreshold;
  int teammatesInPenaltyArea = 0;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(theLibPosition.isNearOpponentPenaltyArea(teammate.getEstimatedPosition(theFrameInfo.time), distanceThreshold, distanceThreshold))
      ++teammatesInPenaltyArea;
  }
  return teammatesInPenaltyArea;
}
