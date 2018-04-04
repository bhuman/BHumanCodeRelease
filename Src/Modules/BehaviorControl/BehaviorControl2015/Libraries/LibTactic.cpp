/**
* @file LibTacticrmation.cpp
*/

#include "../LibraryBase.h"
#include "Platform/SystemCall.h"

namespace Behavior2015
{
  #include "LibTactic.h"
  
  LibTactic::LibTactic():
    xPos(0.0),
    yPos(0.0),
    midPointBallGoal(0.f,0.f),
    angleToOppGoal(0.f),
    angleToOwnGoal(0.f),
    DesiredPos(0.f, 0.f),
    distanceToBall(0.f),
    nbOfKeeper(0),
    nbOfStriker(0),
    nbOfSupporter(0),
    nbOfDefender(0),
    closerToTheBall(false)
  {}
  
  void LibTactic::preProcess() {}

  void LibTactic::postProcess()
  {
    countRoles();
    distanceToBall = abs(sqr(pow(theBallModel.estimate.position.x(), 2) + pow(theBallModel.estimate.position.y(), 2)));
    closerToTheBall = isCloserToTheBall();

    switch(theBehaviorStatus.role)
    {
      case Role::undefined:
      case Role::none:
        //off
        break;
      case Role::keeper:
        midPointBallGoal = (((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f))
                             + Vector2f(theBallModel.estimate.position.x(), theBallModel.estimate.position.y())) / 2);
        xPos = clamp(midPointBallGoal.x(),
                     (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea,
                                                          theFieldDimensions.yPosRightPenaltyArea)).x(),
                     (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline,
                                                          theFieldDimensions.yPosLeftPenaltyArea)).x());
        yPos = clamp(midPointBallGoal.y(),
                     (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea,
                                                          theFieldDimensions.yPosRightPenaltyArea)).y(),
                     (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline,
                                                          theFieldDimensions.yPosLeftPenaltyArea)).y());
        angleToOwnGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
        DesiredPos = Vector2f(xPos, yPos);
        break;
      case Role::defender:
        break;
      case Role::striker:
        angleToOppGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
        break;
      case Role::supporter:
        break;
    }
  }
  
  int LibTactic::timeSinceBallWasSeen()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }
  
  void LibTactic::countRoles()
  {
    // reset count
    this->nbOfDefender = 0;
    this->nbOfKeeper = 0;
    this->nbOfStriker = 0;
    this->nbOfSupporter = 0;
    for(int i = 0; i != theTeammateData.teammates.size(); i++)
    {
      if(theTeammateData.teammates[i].behaviorStatus.role == Role::defender)
      {
        this->nbOfDefender++;
      }
      else if(theTeammateData.teammates[i].behaviorStatus.role == Role::keeper)
      {
        this->nbOfKeeper++;
      }
      else if(theTeammateData.teammates[i].behaviorStatus.role == Role::striker)
      {
        this->nbOfStriker++;
      }
      else if(theTeammateData.teammates[i].behaviorStatus.role == Role::supporter)
      {
        this->nbOfSupporter++;
      }
      else{}
    }
  }    
  
  bool LibTactic::isCloserToTheBall()
  {
    for(int i = 0; i != theTeammateData.teammates.size(); i++)
    {
      if(theTeammateData.teammates[i].behaviorStatus.role == Role::supporter ||
          theTeammateData.teammates[i].behaviorStatus.role == Role::striker)
      {
        double teammateDistanceToBall = abs(sqr(pow(theTeammateData.teammates[i].ball.estimate.position.x(), 2) +
                                                pow(theTeammateData.teammates[i].ball.estimate.position.y(), 2)));

        if(this->distanceToBall < teammateDistanceToBall)
        {
          return false;
        }
      }
    }
    return true;
  }
  
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
