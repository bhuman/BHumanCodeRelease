/**
 * @file LibTeamProvider.cpp
 * @author Lukas Malte Monerjahn
 */

#include "LibTeammatesProvider.h"
#include "Tools/BehaviorControl/Strategy/ActiveRole.h"

MAKE_MODULE(LibTeammatesProvider);

void LibTeammatesProvider::update(LibTeammates& libTeammates)
{
  libTeammates.nonKeeperTeammatesInOwnGoalArea = nonKeeperTeammatesInOwnGoalArea();

  libTeammates.getStrikerBallPosition = [this]() -> Vector2f
  {
    if(theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall))
      return theBallModel.estimate.position;
    else
    {
      for(auto const& teammate : theTeamData.teammates)
        if(teammate.theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall))
          return teammate.theBallModel.estimate.position;
    }
    return Vector2f::Zero();
  };
}

int LibTeammatesProvider::nonKeeperTeammatesInOwnGoalArea() const
{
  float distanceThreshold = outsideDistanceThreshold;
  bool isNear = false;
  if(theLibPosition.isNearOwnGoalArea(theRobotPose.translation, -theRobotPose.getXAxisStandardDeviation(), -theRobotPose.getYAxisStandardDeviation()))
    distanceThreshold = insideDistanceThreshold;
  else if(theLibPosition.isNearOwnGoalArea(theRobotPose.translation, outsideDistanceThreshold, outsideDistanceThreshold))
    isNear = true;
  int teammatesInGoalArea = 0;
  for(auto const& teammate : theTeamData.teammates)
  {
    const Vector2f teammatePosition = teammate.getEstimatedPosition(theFrameInfo.time);
    if(!teammate.isGoalkeeper
       && theLibPosition.isNearOwnGoalArea(teammatePosition, distanceThreshold, distanceThreshold))
    {
      if(isNear && !theLibPosition.isNearOwnGoalArea(teammatePosition, insideDistanceThreshold, insideDistanceThreshold))
      {
        /// breaks ties between two robots attempting to enter simultaneously
        if(teammate.theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall))
          ++teammatesInGoalArea;
      }
      else
        ++teammatesInGoalArea;
    }
  }
  return teammatesInGoalArea;
}
