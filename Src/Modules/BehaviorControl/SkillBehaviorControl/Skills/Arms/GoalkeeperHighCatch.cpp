/**
 * @file GoalkeeperHighCatch.cpp
 *
 * This file implements the GoalkeeperHighCatch skill
 *
 * @author Bastian Trimpf
 */

#include "SkillBehaviorControl.h"

unsigned timestamp;
option((SkillBehaviorControl) GoalkeeperHighCatch)
{
  initial_state(noHighBall)
  {
    transition
    {
      const float oppRange = 400.f; /**<Range around Ball to check for opponent*/

      if(theGameState.isGoalkeeper() && !theGameState.isFreeKick() &&
         theFieldBall.ballWasSeen(1000) && between<float>(std::abs(theFieldBall.positionOnField.x()), std::abs(theFieldDimensions.xPosOwnPenaltyArea), theFieldDimensions.centerCircleRadius))
      {
        for(GlobalOpponentsModel::OpponentEstimate opponent : theGlobalOpponentsModel.opponents) //check if opponent in Position to shoot a (high) shot
        {
          if((theFieldBall.positionOnField - opponent.position).squaredNorm() <= sqr(oppRange))
          {
            goto goalkeeperHighCatch;
          }
        }
      }
    }
  }
  state(goalkeeperHighCatch)
  {
    transition
    {
      const float oppRange = 450.f; /**<Range around ball to check for opponent*/

      for(GlobalOpponentsModel::OpponentEstimate opponent : theGlobalOpponentsModel.opponents) //check if opponent in Position to shoot a (high) shot
      {
        if((theFieldBall.positionOnField - opponent.position).squaredNorm() <= sqr(oppRange))
        {
          timestamp = theFrameInfo.time;
        }
      }
      if(theFrameInfo.getTimeSince(timestamp) >= 1000) //keep up arms up until ball reached goalkeeper
      {
        goto noHighBall;
      }
    }
    action
    {
      //calculate if keeper is near Goalpost
      const auto [isNearLeftPost, isNearRightPost] = theLibPosition.isNearPost(theRobotPose);

      //prevent arms getting caught in Goal and prevent arm movement oustside of Penalty area
      if(theLibPosition.isOutSideGoalFrame(Vector2f(theRobotPose.translation.x(), theRobotPose.translation.y()), 0.f) && theLibPosition.isInOwnPenaltyArea(Vector2f(theRobotPose.translation.x(), theRobotPose.translation.y())))
      {
        isNearLeftPost ? KeyFrameRightArm({.motion = ArmKeyFrameRequest::keeperStand}) :
        isNearRightPost ? KeyFrameLeftArm({.motion = ArmKeyFrameRequest::keeperStand}) :
        KeyFrameArms({.motion = ArmKeyFrameRequest::keeperStand});
      }
    }
  }
}
