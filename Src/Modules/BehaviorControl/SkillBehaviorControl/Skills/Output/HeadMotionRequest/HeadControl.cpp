/**
 * @file HeadControl.cpp
 *
 * This file implements head control skills.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Tools/Modeling/BallPhysics.h"
#include <cmath>

namespace HeadControl
{
  void setPanTiltRequest(HeadMotionRequest& theHeadMotionRequest, HeadMotionRequest::CameraControlMode camera,
                         Angle pan, Angle tilt, Angle speed, bool stopAndGoMode = false, bool calibrationMode = false)
  {
    theHeadMotionRequest.mode = calibrationMode ? HeadMotionRequest::calibrationMode : HeadMotionRequest::panTiltMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.pan = pan;
    theHeadMotionRequest.tilt = tilt;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
  }

  void setTargetOnGroundRequest(HeadMotionRequest& theHeadMotionRequest, HeadMotionRequest::CameraControlMode camera,
                                const Vector3f& target, Angle speed)
  {
    theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.target = target;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = false;
  }
}
using namespace HeadControl;

option((SkillBehaviorControl) LookActive,
       args((bool) withBall,
            (bool) ignoreBall,
            (bool) onlyOwnBall,
            (bool) fixTilt,
            (float) slowdownFactor))
{
  initial_state(execute)
  {
    action
    {
      const HeadTarget target = theLibLookActive.calculateHeadTarget(withBall, ignoreBall, onlyOwnBall, fixTilt, slowdownFactor);
      setPanTiltRequest(theHeadMotionRequest, target.cameraControlMode, target.pan, target.tilt, target.speed, target.stopAndGoMode);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
}

option((SkillBehaviorControl) LookAtAngles,
       args((Angle) pan,
            (Angle) tilt,
            (float) speed,
            (HeadMotionRequest::CameraControlMode) camera,
            (bool) calibrationMode))
{
  initial_state(execute)
  {
    action
    {
      setPanTiltRequest(theHeadMotionRequest, camera, pan, tilt, speed, false, calibrationMode);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
}

option((SkillBehaviorControl) LookAtBall,
       defs((int)(2000) ownBallTimeout, /**< Use the team ball or look active if the ball hasn't been seen for this time. */
            (int)(500) ownBallDisappearedTimeout, /**< Use the team ball or look active if the ball disappeared for this time. */
            (float)(0.2f) propagateBallTime))
{
  const bool useOwnEstimate = theFieldBall.ballWasSeen(ownBallTimeout) && theFieldBall.timeSinceBallDisappeared <= ownBallDisappearedTimeout;

  common_transition
  {
    if(useOwnEstimate || theTeammatesBallModel.isValid)
      goto lookAtBall;
    else
      goto lookActive;
  }

  initial_state(lookAtBall)
  {
    action
    {
      const Vector2f ballPosition = theFieldBall.recentBallPropagatedPositionRelative(propagateBallTime, theBallSpecification.friction);
      setTargetOnGroundRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera,
                               {ballPosition.x(), ballPosition.y(), theBallSpecification.radius}, 180_deg);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }

  state(lookActive)
  {
    action
    {
      const HeadTarget target = theLibLookActive.calculateHeadTarget(false, false, false, false, 1.f);
      setPanTiltRequest(theHeadMotionRequest, target.cameraControlMode, target.pan, target.tilt, target.speed, target.stopAndGoMode);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
}

option((SkillBehaviorControl) LookAtPoint,
       args((const Vector3f&) target,
            (HeadMotionRequest::CameraControlMode) camera,
            (Angle) speed))
{
  initial_state(execute)
  {
    action
    {
      setTargetOnGroundRequest(theHeadMotionRequest, camera, target, speed);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
}

option((SkillBehaviorControl) LookForward)
{
  initial_state(execute)
  {
    action
    {
      setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, 0.f, 0.38f, 150_deg);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
}

option((SkillBehaviorControl) LookLeftAndRight,
       args((bool) startLeft,
            (Angle) maxPan,
            (Angle) tilt,
            (Angle) speed))
{
  initial_state(execute)
  {
    transition
    {
      if(startLeft)
        goto left;
      else
        goto right;
    }
  }

  state(left)
  {
    transition
    {
      if(!theHeadMotionInfo.moving && std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - maxPan)) < 5_deg)
        goto right;
    }
    action
    {
      setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, maxPan, tilt, speed);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }

  state(right)
  {
    transition
    {
      if(!theHeadMotionInfo.moving && std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] + maxPan)) < 5_deg)
        goto left;
    }
    action
    {
      setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, -maxPan, tilt, speed);
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
}

option((SkillBehaviorControl) LookAtBallAndTarget,
       args((bool) startBall, /**< If true, then look at the ball first. */
            (Angle) tilt,
            (Angle) thresholdAngle,
            (Vector2f) walkingDirection),
       defs((int)(100) minTimeBetweenTargetSwitch, /**< Switch from walk to ball target only after this time passed (in ms) */
            (int)(200) minTimeLookingAtBall, /**< Switch from ball to walk target only after this time passed (in ms) */
            (int)(1000) maxWalkTargetTime, /**< Do not stay in look at walk target state longer than this time (in s) */
            (float)(0.2f) propagateBallTime, /**< Propagate the ball position this time into the future (in s) */
            (Angle)(75_deg) borderAngle, /**< Max head angle. */
            (Angle)(30_deg) searchBallAngle, /**< Max head angle when searching for the ball. */
            (Angle)(500_deg) headSpeedFast,
            (Angle)(100_deg) headSpeedNormal,
            (Angle)(50_deg) headSpeedSlow),
       vars((Angle)(theJointAngles.angles[Joints::headYaw]) nextTurnedAngle, /**< The last angle the head in LookAtBallTarget was turned */
            (unsigned)(0) lastTargetSwitch, /**< Timestamp of last target switch. */
            (Angle)(theJointAngles.angles[Joints::headYaw]) lastHeadYawAngle, /**< Last head yaw angle. */
            (unsigned)(theFrameInfo.time) lookingAtBallSince, /**< Timestamp since looking at ball. */
            (unsigned)(theJointAngles.timestamp) lastJointAnglesTime, /**< Timestamp of last measured joint angles. */
            (bool)(false) searchForBall, /**< When moving from walk target to ball, but the ball is not seen, look further. */
            (bool)(false) isSearchingForBall, /**< Currently looking for the ball. */
            (float)(1.f) searchBallLeft)) /**< Which side to search for the ball. */
{
  const Vector2f ballPosition = theFieldBall.recentBallPropagatedPositionRelative(propagateBallTime, theBallSpecification.friction);
  Angle ballPositionAngle = ballPosition.angle();
  Angle walkingAngle = Rangea(-borderAngle, borderAngle).limit(walkingDirection.angle()); // Angle of the walking direction related to the robot
  ballPositionAngle = Rangea(-borderAngle, borderAngle).limit(ballPositionAngle); // angle between robot position and ball position
  const float ratio = Rangef::ZeroOneRange().limit(walkingDirection.norm() / 500.f); //todo: threshold has to be adjusted
  walkingAngle = ballPositionAngle * (1.f - ratio) + walkingAngle * ratio;

  initial_state(execute)
  {
    transition
    {
      if(startBall)
      {
        if(std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - ballPositionAngle)) < 45_deg)
          goto lookAtBall;
        goto lookAtMiddle;
      }
      goto lookAtWalkTarget;
    }
  }
  state(lookAtWalkTarget)
  {
    transition
    {
      const bool headNearTarget = std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - nextTurnedAngle)) <= thresholdAngle;
      if(std::abs(walkingAngle - ballPositionAngle) <= thresholdAngle || theBallModel.estimate.velocity.squaredNorm() > 0 ||  //todo: oscillation
         (!theHeadMotionInfo.moving && theFrameInfo.getTimeSince(lastTargetSwitch) > minTimeBetweenTargetSwitch && headNearTarget) ||
         theFrameInfo.getTimeSince(lastTargetSwitch) > maxWalkTargetTime)
      {
        if(std::abs(walkingAngle - ballPositionAngle) <= thresholdAngle || theBallModel.estimate.velocity.squaredNorm() > 0)
          goto lookAtBall;
        goto lookAtMiddle;
      }
    }
    action
    {
      if(state_time == 0)
      {
        nextTurnedAngle = Angle(walkingDirection.angle());
        lastTargetSwitch = theFrameInfo.time;
      }

      searchForBall = !theFieldBall.ballWasSeen();
      nextTurnedAngle = walkingAngle;
      setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, nextTurnedAngle, tilt, headSpeedFast);
      lastHeadYawAngle = theJointAngles.angles[Joints::headYaw];
      lastJointAnglesTime = theJointAngles.timestamp;
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
  state(lookAtBall)
  {
    transition
    {
      const bool headNearTarget = std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - nextTurnedAngle)) <= thresholdAngle;
      if(!theHeadMotionInfo.moving && theFrameInfo.getTimeSince(lastTargetSwitch) > minTimeBetweenTargetSwitch &&
         theFrameInfo.getTimeSince(lookingAtBallSince) > minTimeLookingAtBall && headNearTarget && !searchForBall && !isSearchingForBall)
        goto lookAtWalkTarget;
    }
    action
    {
      if(state_time == 0)
      {
        nextTurnedAngle = ballPositionAngle;
        lastTargetSwitch = theFrameInfo.time;
        lookingAtBallSince = theFrameInfo.time;
        searchBallLeft = ballPositionAngle - walkingAngle < 0.f ? -1.f : 1.f;
      }
      if((searchForBall || isSearchingForBall) && std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - nextTurnedAngle)) <= thresholdAngle)
      {
        searchForBall = false;
        isSearchingForBall = !isSearchingForBall && !theFieldBall.ballWasSeen(); // stop searching when ball is seen
      }
      if(isSearchingForBall) // stop searching when ball is seen
        isSearchingForBall = !theFieldBall.ballWasSeen();
      nextTurnedAngle = ballPositionAngle + (isSearchingForBall ? Angle(searchBallLeft * searchBallAngle) : 0_deg);
      setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, nextTurnedAngle, tilt, isSearchingForBall ? headSpeedSlow : headSpeedNormal); // move head slower when searching for ball
      lastHeadYawAngle = theJointAngles.angles[Joints::headYaw];
      lastJointAnglesTime = theJointAngles.timestamp;
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }

  state(lookAtMiddle)
  {
    transition
    {
      const bool headNearTarget = std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - nextTurnedAngle)) <= thresholdAngle;
      if(theFrameInfo.getTimeSince(lastTargetSwitch) > maxWalkTargetTime ||
         (!theHeadMotionInfo.moving && theFrameInfo.getTimeSince(lastTargetSwitch) > minTimeBetweenTargetSwitch && headNearTarget))
        goto lookAtBall;
    }
    action
    {
      if(state_time == 0)
      {
        nextTurnedAngle = (nextTurnedAngle + ballPositionAngle) / 2.f;
        lastTargetSwitch = theFrameInfo.time;
      }
      searchForBall = !theFieldBall.ballWasSeen();
      setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, nextTurnedAngle, tilt, headSpeedFast);
      lastHeadYawAngle = theJointAngles.angles[Joints::headYaw];
      lastJointAnglesTime = theJointAngles.timestamp;
      theLibCheck.inc(LibCheck::headMotionRequest);
    }
  }
}
