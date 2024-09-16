#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleIllegalAreas)
{
  const Angle margin = 10_deg;
  const int durationUntilAnticipatedIllegal = (theGameState.timeWhenStateEnds - theGameState.timeWhenStateStarted) / 2;

  auto approximateTarget = [&]
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
              theIllegalAreas.willPositionBeIllegalIn(theRobotPose * approximateTarget(), 150.f, durationUntilAnticipatedIllegal))
        goto waiting;
    }
  }

  state(illegal)
  {
    transition
    {
      if(!theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 300.f, durationUntilAnticipatedIllegal))
      {
        if(!theIllegalAreas.willPositionBeIllegalIn(theRobotPose * approximateTarget(), 300.f, durationUntilAnticipatedIllegal))
          goto notIllegal;
        goto waiting;
      }
    }

    action
    {
      theLibCheck.dec(LibCheck::motionRequest);
      theLibCheck.dec(LibCheck::headMotionRequest);

      // Reset BehaviorStatus::walkingTo.
      PublishMotion({.target = Vector2f::Zero()});

      if(theFieldBall.ballWasSeen(7000) || theTeammatesBallModel.isValid)
      {
        WalkPotentialField({.target = theRobotPose.translation,
                            .straight = true,
                            .targetOfInterest = theFieldBall.recentBallPositionRelative()});
      }
      else
      {
        WalkPotentialField({.target = theRobotPose.translation,
                            .straight = true});
        LookActive({.withBall = lookAtBall()});
      }
    }
  }

  state(waiting)
  {
    transition
    {
      if(!theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 300.f, durationUntilAnticipatedIllegal) &&
         !theIllegalAreas.willPositionBeIllegalIn(theRobotPose * approximateTarget(), 300.f, durationUntilAnticipatedIllegal))
        goto notIllegal;
      if(theIllegalAreas.willPositionBeIllegalIn(theRobotPose.translation, 150.f, durationUntilAnticipatedIllegal))
        goto illegal;
    }

    action
    {
      theLibCheck.dec(LibCheck::motionRequest);
      theLibCheck.dec(LibCheck::headMotionRequest);

      // Reset BehaviorStatus::walkingTo.
      PublishMotion({.target = Vector2f::Zero()});

      const Vector2f targetRelative = approximateTarget();
      if(Rangea(-margin, margin).isInside(targetRelative.angle()))
        Stand();
      else
        TurnToPoint({.target = targetRelative});
      LookActive({.withBall = lookAtBall()});
    }
  }
}
