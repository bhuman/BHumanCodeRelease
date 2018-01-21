/**
* @file LibInformation.cpp
*/

#include "../LibraryBase.h"

namespace Behavior2015
{
  #include "LibInfo.h"
  
  LibInfo::LibInfo():
    angleToOppGoal(0.f),
    angleToOwnGoal(0.f),
    keeperDesiredPos(0.f, 0.f),
    SupporterDesiredPos(0.f,0.f),
    distanceToBall(0.f),
    nbOfDef(0),
    closerToTheBall(false)
  {}
  
  void LibInfo::preProcess()
  {
    nbOfDef = howManyDef();
    switch(WalkingEngineState.role)
    {
    case keeper:
      Vector2f midPointBallGoal = (((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)) + Vector2f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y())) / 2);
      float xPos = clamp(midPointBallGoal.x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).x());
      float yPos = clamp(midPointBallGoal.y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).y());
      angleToOwnGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
      keeperDesiredPos = Vector2f(xPos, yPos);
      SystemCall::playSound("helpMe.wav");
      break;
    case defender:
      SystemCall::playSound("theFlippingChicken.wav");
      break;
    case striker:
      angleToOppGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
      distanceToBall = sqr(theBallModel.estimate.position.x()^2 + theBallModel.estimate.position.y()^2) 
      closerToTheBall = isCloserToTheBall();
      SystemCall::playSound("falling.wav");
      break;
    case supporter:
      distanceToBall = sqr(theBallModel.estimate.position.x()^2 + theBallModel.estimate.position.y()^2) 
      closerToTheBall = isCloserToTheBall();
      SystemCall::playSound("theMirrorCow.wav");
      break;
    default:
      break;
    }  
  }

  void LibInfo::postProcess()
  {
  }
  
  int LibInfo::timeSinceBallWasSeen()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }
  
  int LibInfo::howManyDef()
  {
    int nb = 0;
    for(int i = 0; i != theTeammateData.size(); i++)
    {
      if(theTeammateData.teammates[i].behaviorStatus.role == Role::defender)
      {
        nb++;
      }
    }
    return nb;
  }    
  
  bool LibInfo::isCloserToTheBall()
  {
    for(int i = 0; i != theTeammateData.size(); i++)
    {
      if(theTeammateData.teammates[i].behaviorStatus.role == Role::supporter || theTeammateData.teammates[i].behaviorStatus.role == Role::striker)
      {
          tmDistanceToBall = sqr(theTeammateData.teammates[i].theBallModel.estimate.position.x()^2 + theTeammateData.teammates[i].theBallModel.estimate.position.y()^2) 
      
        if(tmDistanceToBall < distanceToBall)
        {
          return false;
        }
      }
    }
    return true;
  }
  
  bool LibInfo::between(float value, float min, float max)
  {
    return value >= min && value <= max;
  }

  float LibInfo::clamp(float value, float min, float max)
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
