/**
* @file LibCodeRelease.cpp
*/

#include "../LibraryBase.h"

namespace Behavior2015
{
  #include "LibCodeRelease.h"
  
  LibCodeRelease::LibCodeRelease():
    angleToOppGoal(0.f),
    angleToOwnGoal(0.f),
    KeeperDesiredPos(0.f, 0.f)
  {}
  
  void LibCodeRelease::preProcess()
  {
    Vector2f middleBallGoal = (((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)) + Vector2f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y())) / 2);
    float xPos = clamp(middleBallGoal.x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).x());
 //clamp(theBallModel.estimate.position.x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).x());
    float yPos = clamp(middleBallGoal.y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).y());
    //clamp(theBallModel.estimate.position.y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).y());
          
    angleToOppGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    angleToOwnGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
    KeeperDesiredPos = Vector2f(xPos, yPos);
  }

  void LibCodeRelease::postProcess()
  {
  }
  
  int LibCodeRelease::timeSinceBallWasSeen()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }
  
  bool LibCodeRelease::between(float value, float min, float max)
  {
    return value >= min && value <= max;
  }

  float LibCodeRelease::clamp(float value, float min, float max)
  {
    if(min > max)
    {
        float tmp = max;
        max = min;
        min = tmp;
    }
      
    if(value <= min)
    {
        return min;
    }
    else if(value >= max)
    {
        return max;
    }
    else
    {
        return value;
    }
  }    
}