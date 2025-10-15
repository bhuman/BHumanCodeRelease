#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleRollingBallChallenge,
       defs((Angle)(-10_deg) kickAngle,
            (Angle)(3_deg) minRotError,
            (float)(1400.f) minWaitingDistance,  /**< The minimum ball distance for staying in waiting and positioning mode. */
            (float)(1500.f) maxKickingDistance, /**< The maximum ball distance for staying in kicking mode. */
            (int)(2000) maxUnseenTime, /**< The maximum time period for staying in kicking mode without seeing the ball. */
            (Rangea)(-10_deg, 10_deg) directionPrecision), /**< The desired kick precision. */
       vars((Pose2f)(theOdometryData) startOdometry)) /**< The odometry at start. */
{
  const Angle waitingRot = (theRollingBallState.isRampLeftSide ? -1.f : 1.f) * kickAngle;
  const Pose2f odometryDiff = theOdometryData - startOdometry;

  initial_state(walkToWaitingPose)
  {
    transition
    {
      if(theRollingBallState.isRampLeftSide)
        goto turnLeft;
      else
        goto turnRight;
    }
    action
    {
      HandleHeadInRollingBallChallenge();
      Stand({.energySavingWalk = false});
      theMotionRequest.shouldInterceptBall = false;
    }
  }

  state(standing)
  {
    transition
    {
      if(std::abs(theFieldBall.positionOnField.y()) < minWaitingDistance &&
         (theFieldInterceptBall.interceptBall || theFieldInterceptBall.predictedInterceptBall))
      {
        if(theRollingBallState.isRampLeftSide)
          goto kickBallLeft;
        else
          goto kickBallRight;
      }
    }
    action
    {
      HandleHeadInRollingBallChallenge();
      Stand({.energySavingWalk = false});
      theMotionRequest.shouldInterceptBall = false;
    }
  }

  state(turnLeft)
  {
    transition
    {
      const Angle waitPose = waitingRot - odometryDiff.rotation;
      if(waitPose < minRotError || state_time > 2000)
        goto standing;
    }
    action
    {
      const Angle waitPose = waitingRot - odometryDiff.rotation;
      const float turnSpeed = mapToRange(std::abs(waitPose), std::abs(kickAngle), std::abs(minRotError), 0.3f, 0.1f);
      WalkAtRelativeSpeed({.speed = Pose2f(turnSpeed, 0.f, 0.f)});
      HandleHeadInRollingBallChallenge();
      theMotionRequest.shouldInterceptBall = false;
    }
  }

  state(turnRight)
  {
    transition
    {
      const Angle waitPose = waitingRot - odometryDiff.rotation;
      if(waitPose > -minRotError || state_time > 2000)
        goto standing;
    }
    action
    {
      const Angle waitPose = waitingRot - odometryDiff.rotation;
      const float turnSpeed = mapToRange(std::abs(waitPose), std::abs(kickAngle), std::abs(minRotError), 0.3f, 0.1f);
      WalkAtRelativeSpeed({.speed = Pose2f(-turnSpeed, 0.f, 0.f)});
      HandleHeadInRollingBallChallenge();
      theMotionRequest.shouldInterceptBall = false;
    }
  }

  state(kickBallRight)
  {
    transition
    {
      if(std::abs(theFieldBall.positionOnField.y()) > maxKickingDistance ||
         theFieldBall.timeSinceBallWasSeen > maxUnseenTime ||
         !(theFieldInterceptBall.interceptBall || theFieldInterceptBall.predictedInterceptBall))
        goto standing;
    }
    action
    {
      HandleHeadInRollingBallChallenge();
      WalkToBallAndKick({.targetDirection = -theKickInfo[KickInfo::walkForwardsRightLong].rotationOffset,
                         .kickType = KickInfo::walkForwardsRightLong,
                         .alignPrecisely = KickPrecision::justHitTheBall,
                         .directionPrecision = directionPrecision});
    }
  }

  state(kickBallLeft)
  {
    transition
    {
      if(std::abs(theFieldBall.positionOnField.y()) > maxKickingDistance ||
         theFieldBall.timeSinceBallWasSeen > maxUnseenTime ||
         !(theFieldInterceptBall.interceptBall || theFieldInterceptBall.predictedInterceptBall))
        goto standing;
    }
    action
    {
      HandleHeadInRollingBallChallenge();
      WalkToBallAndKick({.targetDirection = -theKickInfo[KickInfo::walkForwardsLeftLong].rotationOffset,
                         .kickType = KickInfo::walkForwardsLeftLong,
                         .alignPrecisely = KickPrecision::justHitTheBall,
                         .directionPrecision = directionPrecision});
    }
  }
}

option((SkillBehaviorControl) HandleHeadInRollingBallChallenge,
       defs((int)(1000) maxUnseenTime, /**< The maximum time period for looking at the ball's position without seeing it. */
            (int)(50) minUnseenTime, /**< The Minimum time period for looking at the ball's position without seeing it, to switch to lookAtBallWaiting. */
            (float)(400.f) ballNormDistance))
{
  // The absolute position to look at if the ball position is unknown
  const Vector2f searchBallPoint(750.f, (theRollingBallState.isRampLeftSide ? 1.f : -1.f) * theRollingBallState.approxRampDistance);

  auto shouldLookAtBall = [&](const int unseenTime) -> bool
  {
    return theFieldBall.timeSinceBallWasSeen <= unseenTime &&
    Geometry::getDistanceToLine(Geometry::Line(searchBallPoint, Vector2f(0.f, -searchBallPoint.y())), theFieldBall.positionOnField) < ballNormDistance && (
      theFieldBall.positionOnField.y() * searchBallPoint.y() > 0.f ||
      theBallModel.estimate.velocity != Vector2f::Zero());
  };

  initial_state(lookAtPoint)
  {
    transition
    {
      if(shouldLookAtBall(maxUnseenTime))
        goto lookAtBall;
    }
    action
    {
      const Vector2f fieldPoint = theRobotPose.inverse() * searchBallPoint;
      LookAtPoint({.target = {fieldPoint.x(), fieldPoint.y(), 0.f}, .camera = HeadMotionRequest::lowerCamera, .speed = 40_deg});
    }
  }

  state(lookAtBall)
  {
    transition
    {
      if(!shouldLookAtBall(minUnseenTime))
        goto lookAtBallWaiting;
    }
    action
    {
      LookAtBall({.camera = HeadMotionRequest::lowerCamera});
    }
  }

  state(lookAtBallWaiting)
  {
    transition
    {
      if(!shouldLookAtBall(maxUnseenTime) && state_time > maxUnseenTime)
        goto lookAtPoint;
      else if(shouldLookAtBall(maxUnseenTime))
        goto lookAtBall;
    }
    action
    {
      LookAtBall({.camera = HeadMotionRequest::lowerCamera});
    }
  }
}
