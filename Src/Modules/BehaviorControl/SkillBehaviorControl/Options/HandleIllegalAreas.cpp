#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleIllegalAreas)
{
  const Angle margin = 30_deg;
  const int durationUntilAnticipatedIllegal = (theGameState.timeWhenStateEnds - theGameState.timeWhenStateStarted) / 2;

  auto approximateWalkTarget = [&]
  {
    Vector2f targetPosition = Vector2f::Zero();
    switch(theMotionRequest.motion)
    {
      case MotionRequest::stand:
        break;
      case MotionRequest::walkToPose:
        targetPosition = theMotionRequest.walkTarget.translation;
        break;
      case MotionRequest::walkToBallAndKick:
      case MotionRequest::dribble:
        targetPosition = theMotionRequest.ballEstimate.position;
        break;
      default:
        break;
    }
    return targetPosition;
  };

  auto approximateVisionTarget = [&]
  {
    Vector2f targetPosition = Vector2f::Zero();
    if(theMotionRequest.targetOfInterest.has_value())
      return theMotionRequest.targetOfInterest.value();
    switch(theMotionRequest.motion)
    {
      case MotionRequest::stand:
        break;
      case MotionRequest::walkToPose:
        targetPosition = theMotionRequest.walkTarget.translation;
        break;
      case MotionRequest::walkToBallAndKick:
      case MotionRequest::dribble:
        targetPosition = theMotionRequest.ballEstimate.position;
        break;
      default:
        break;
    }
    return targetPosition;
  };

  auto lookAtBall = [&]
  {
    switch(theMotionRequest.motion)
    {
      case MotionRequest::walkToBallAndKick:
      case MotionRequest::dribble:
        return true;
      default:
        break;
    }
    return false;
  };

  initial_state(notIllegal)
  {
    transition
    {
      if(theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 150.f, durationUntilAnticipatedIllegal))
        goto illegal;
      else if(theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 300.f, durationUntilAnticipatedIllegal) &&
              theIllegalAreas.willPositionBeIllegalIn(theRobotPose * approximateWalkTarget(), 150.f, durationUntilAnticipatedIllegal))
        goto waiting;
    }
  }

  state(illegal)
  {
    transition
    {
      if(!theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 300.f, durationUntilAnticipatedIllegal))
      {
        if(!theIllegalAreas.willPositionBeIllegalIn(theRobotPose * approximateWalkTarget(), 300.f, durationUntilAnticipatedIllegal))
          goto notIllegal;
        goto waiting;
      }
    }

    action
    {
      theLibCheck.dec(LibCheck::motionRequest);
      theLibCheck.dec(LibCheck::headMotionRequest);

      // Reset BehaviorStatus::walkingTo.
      PublishMotion({.target = {0.f, 0.f}});

      if(theFieldBall.ballWasSeen(7000) || theTeamBallModel.isValid)
      {
        WalkPotentialField({.target = theRobotPose.translation,
                            .straight = true,
                            .targetOfInterest = theFieldBall.recentBallPositionRelative()});
      }
      else
      {
        LookActive({.withBall = lookAtBall()});
        WalkPotentialField({.target = theRobotPose.translation,
                            .straight = true});
      }
    }
  }

  state(waiting)
  {
    transition
    {
      if(!theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 300.f, durationUntilAnticipatedIllegal) &&
         !theIllegalAreas.willPositionBeIllegalIn(theRobotPose * approximateWalkTarget(), 300.f, durationUntilAnticipatedIllegal))
        goto notIllegal;
      if(theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 150.f, durationUntilAnticipatedIllegal))
        goto illegal;
    }

    action
    {
      theLibCheck.dec(LibCheck::motionRequest);
      theLibCheck.dec(LibCheck::headMotionRequest);

      // Reset BehaviorStatus::walkingTo.
      PublishMotion({.target = {0.f, 0.f}});

      const Vector2f targetRelative = approximateVisionTarget();
      LookActive({.withBall = lookAtBall()});
      if(Rangea(-margin, margin).isInside(targetRelative.angle()))
        Stand();
      else
        TurnToPoint({.target = targetRelative});
    }
  }
}
