/**
* @file LibTacticrmation.cpp
*/

#include "../LibraryBase.h"
#include "Platform/SystemCall.h"

namespace Behavior2015
{
  #include "LibTactic.h"
  
  LibTactic::LibTactic():
    angleToOppGoal(0.f),
    angleToOwnGoal(0.f),
    DesiredPos(0.f, 0.f),
    distanceToBall(0.f),
    nbOfDef(0),
    closerToTheBall(false)
  {}
  
  void LibTactic::preProcess()
  {
    nbOfDef = howManyDef();
    distanceToBall = abs(sqr(pow(theBallModel.estimate.position.x(),2) + pow(theBallModel.estimate.position.y(),2)));
    closerToTheBall = isCloserToTheBall();
    if(theBehaviorStatus.role == Role::keeper)
    {
      Vector2f midPointBallGoal = (((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)) + Vector2f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y())) / 2);
      float xPos = clamp(midPointBallGoal.x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).x(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).x());
      float yPos = clamp(midPointBallGoal.y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea)).y(), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea)).y());
      angleToOwnGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
      DesiredPos = Vector2f(xPos, yPos);
    }
    else if(theBehaviorStatus.role == Role::defender)
    {
    }
    else if(theBehaviorStatus.role == Role::striker)
    {

      angleToOppGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    }
    else if(theBehaviorStatus.role == Role::supporter)
    {

    }
    else
    {
    }
  }

  void LibTactic::postProcess(){}
  
  int LibTactic::timeSinceBallWasSeen()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }
  
  int LibTactic::howManyDef()
  {
    int nb = 0;
    for(int i = 0; i != theTeammateData.teammates.size(); i++)
    {
      if(theTeammateData.teammates[i].behaviorStatus.role == Role::defender)
      {
        nb++;
      }
    }
    return nb;
  }    
  
  bool LibTactic::isCloserToTheBall()
  {
    for(int i = 0; i != theTeammateData.teammates.size(); i++)
    {
      if(theTeammateData.teammates[i].behaviorStatus.role == Role::supporter || theTeammateData.teammates[i].behaviorStatus.role == Role::striker)
      {
        double teammateDistanceToBall = abs(sqr(pow(theTeammateData.teammates[i].ball.estimate.position.x(),2) + pow(theTeammateData.teammates[i].ball.estimate.position.y(),2))); 
      
        if(this->distanceToBall < teammateDistanceToBall)
        {
          return false;
        }
      }
    }
    return true;
  }
//
//  Vector2f LibTactic::findStrikerPos()
//  {
//    for(int i = 0; i != theTeammateData.teammates.size(); i++)
//    {
//      if(theTeammateData.teammates[i].behaviorStatus.role == Role::striker)
//      {
//  }
  
  bool LibTactic::between(float value, float min, float max)
  {
    return value >= min && value <= max;
  }

  float LibTactic::clamp(float value, float min, float max)
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
