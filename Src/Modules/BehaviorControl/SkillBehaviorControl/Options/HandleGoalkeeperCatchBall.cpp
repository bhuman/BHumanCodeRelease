#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/Interception.h"

option((SkillBehaviorControl) HandleGoalkeeperCatchBall)
{
  initial_state(notCatching)
  {
    transition
    {
      if(theGameState.isGoalkeeper() &&
         between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.3f, 3.f) &&
         theFieldBall.ballWasSeen(100) &&
         theFieldBall.isRollingTowardsOwnGoal &&
         theFieldBall.positionRelative.squaredNorm() < sqr(3000.f))
        goto preparingCatch;
    }
  }

  state(preparingCatch)
  {
    transition
    {
      if(!(between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 4.f) &&
           theFieldBall.ballWasSeen(100) &&
           theFieldBall.isRollingTowardsOwnGoal &&
           theFieldBall.positionRelative.squaredNorm() < sqr(3000.f)))
        goto notCatching;
      if(state_time >= 100)
        goto doingCatch;
    }
    action
    {
      LookAtBall();
      KeyFrameArms({.motion = ArmKeyFrameRequest::keeperStand});
      Stand();
    }
  }

  state(doingCatch)
  {
    transition
    {
      if(action_done || !theFieldInterceptBall.interceptBall)
        goto notCatching;
    }
    action
    {
      //calculate isNearPost
      const auto [isNearLeftPost, isNearRightPost] = theLibPosition.isNearPost(theRobotPose);

      unsigned interceptionMethods = bit(Interception::stand) | bit(Interception::walk);
      interceptionMethods |= bit(Interception::genuflectStandDefender);
      if(theLibPosition.isInOwnPenaltyArea(theRobotPose.translation))
      {
        if(!isNearLeftPost)
          interceptionMethods |= bit(Interception::jumpLeft);
        if(!isNearRightPost)
          interceptionMethods |= bit(Interception::jumpRight);
      }

      InterceptBall({.interceptionMethods = interceptionMethods,
                     .allowDive = theBehaviorParameters.keeperJumpingOn});
    }
  }
}
