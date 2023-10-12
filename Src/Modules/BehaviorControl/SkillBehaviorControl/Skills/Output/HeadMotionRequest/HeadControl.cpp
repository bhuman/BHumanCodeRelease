/**
 * @file HeadControl.cpp
 *
 * This file implements head control skills.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibLookActive.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include <cmath>

SKILL_IMPLEMENTATION(HeadControlImpl,
{,
  IMPLEMENTS(LookActive),
  IMPLEMENTS(LookAtAngles),
  IMPLEMENTS(LookAtBall),
  IMPLEMENTS(LookAtGlobalBall),
  IMPLEMENTS(LookAtPoint),
  IMPLEMENTS(LookForward),
  IMPLEMENTS(LookLeftAndRight),
  IMPLEMENTS(LookAtBallAndTarget),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FrameInfo),
  REQUIRES(HeadMotionInfo),
  REQUIRES(JointAngles),
  REQUIRES(LibCheck),
  REQUIRES(LibLookActive),
  REQUIRES(RobotPose),
  REQUIRES(TeammatesBallModel),
  MODIFIES(HeadMotionRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) ownBallTimeout, /**< LookAtBall will use the team ball or look active if the ball hasn't been seen for this time. */
    (int)(500) ownBallDisappearedTimeout, /**< LookAtBall will use the team ball or look active if the ball disappeared for this time. */
    (int)(100) minTimeBetweenTargetSwitch,
    (int)(200) minTimeLookingAtBall,
    (Angle)(100_deg) maxHeadYawSpeed,
  }),
});

class HeadControlImpl : public HeadControlImplBase
{
  void execute(const LookActive& p) override
  {
    const HeadTarget target = theLibLookActive.calculateHeadTarget(p.withBall, p.ignoreBall, p.onlyOwnBall, p.fixTilt);
    setPanTiltRequest(target.cameraControlMode, target.pan, target.tilt, target.speed, target.stopAndGoMode);
  }

  void execute(const LookAtAngles& p) override
  {
    setPanTiltRequest(p.camera, p.pan, p.tilt, p.speed, false, p.calibrationMode);
  }

  void execute(const LookAtBall& p) override
  {
    const bool useOwnEstimate = p.forceOwnEstimate || (theFieldBall.ballWasSeen(ownBallTimeout) && theFieldBall.timeSinceBallDisappeared <= ownBallDisappearedTimeout);
    if(useOwnEstimate || theTeammatesBallModel.isValid)
    {
      const Vector2f ballPosition = useOwnEstimate ?
                                    (p.mirrored ? (theRobotPose.inverse() * (theRobotPose * theBallModel.estimate.position).rotated(pi)) : theBallModel.estimate.position) :
                                    (theRobotPose.inverse() * (p.mirrored ? theTeammatesBallModel.position.rotated(pi) : theTeammatesBallModel.position));
      setTargetOnGroundRequest(HeadMotionRequest::autoCamera, Vector3f(ballPosition.x(), ballPosition.y(), theBallSpecification.radius), 180_deg);
    }
    else
    {
      const HeadTarget target = theLibLookActive.calculateHeadTarget(false, false, false, false);
      setPanTiltRequest(target.cameraControlMode, target.pan, target.tilt, target.speed, target.stopAndGoMode);
    }
  }

  void execute(const LookAtGlobalBall& p) override
  {
    if(theTeammatesBallModel.isValid)
    {
      const Vector2f ballPosition = theRobotPose.inverse() * (p.mirrored ? theTeammatesBallModel.position.rotated(pi) : theTeammatesBallModel.position);
      setTargetOnGroundRequest(HeadMotionRequest::autoCamera, Vector3f(ballPosition.x(), ballPosition.y(), theBallSpecification.radius), 180_deg);
    }
    else
    {
      const HeadTarget target = theLibLookActive.calculateHeadTarget(false, false, false, false);
      setPanTiltRequest(target.cameraControlMode, target.pan, target.tilt, target.speed, target.stopAndGoMode);
    }
  }

  void execute(const LookAtPoint& p) override
  {
    setTargetOnGroundRequest(p.camera, p.target, p.speed);
  }

  void execute(const LookForward&) override
  {
    setPanTiltRequest(HeadMotionRequest::autoCamera, 0.f, 0.38f, 150_deg);
  }

  void execute(const LookLeftAndRight& p) override
  {
    if(!theHeadMotionInfo.moving && std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - lookLeftAndRightSign * p.maxPan)) < 5_deg)
      lookLeftAndRightSign = -lookLeftAndRightSign;
    setPanTiltRequest(HeadMotionRequest::autoCamera, lookLeftAndRightSign * p.maxPan, p.tilt, p.speed);
  }

  //TODO: Clip values for the angles
  void execute(const LookAtBallAndTarget& p) override
  {
    Angle borderAngle = 75_deg; // Angle that limits the movement of the head to prevent getting stuck when not reaching the target position
    Angle walkingAngle = Rangea(-borderAngle, borderAngle).limit(p.walkingDirection.angle()); // Angle of the walking direction related to the robot
    Angle ballPositionAngle = Rangea(-borderAngle, borderAngle).limit(p.ballPositionAngle); // angle between robot position and ball position
    const float ratio = Rangef::ZeroOneRange().limit(p.walkingDirection.norm() / 500.f); //todo: threshold has to be adjusted
    walkingAngle = ballPositionAngle * (1.f - ratio) + walkingAngle * ratio;

    const bool headNearTarget = std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - nextTurnedAngle)) <= p.thresholdAngle;
    const Angle maxHeadYawChange = (theJointAngles.timestamp - lastJointAnglesTime) / 1000.f * maxHeadYawSpeed;
    const bool headMovedSlow = std::abs(lastHeadYawAngle - theJointAngles.angles[Joints::headYaw]) <= maxHeadYawChange;
    if(!lookAtBallFirst || !(headNearTarget && headMovedSlow))
      lookingAtBallSince = theFrameInfo.time;

    //look at the ball if we also see the target or the ball is moving
    if(std::abs(walkingAngle - ballPositionAngle) <= p.thresholdAngle || theBallModel.estimate.velocity.squaredNorm() > 0)//todo: oszillation
    {
      nextTurnedAngle = ballPositionAngle;
      lookAtBallFirst = true;
    }
    else if(!theHeadMotionInfo.moving && theFrameInfo.getTimeSince(lastTargetSwitch) > minTimeBetweenTargetSwitch &&
            (theFrameInfo.getTimeSince(lookingAtBallSince) > minTimeLookingAtBall || !lookAtBallFirst) && headNearTarget)
    {
      lookAtBallFirst = !lookAtBallFirst;
      lastTargetSwitch = theFrameInfo.time;
    }
    if(lookAtBallFirst)
      nextTurnedAngle = ballPositionAngle;
    else
      nextTurnedAngle = walkingAngle;

    setPanTiltRequest(HeadMotionRequest::autoCamera, nextTurnedAngle, p.tilt, 500_deg);
    lastHeadYawAngle = theJointAngles.angles[Joints::headYaw];
    lastJointAnglesTime = theJointAngles.timestamp;
  }

  void reset(const LookActive&) override {}
  void reset(const LookAtAngles&) override {}
  void reset(const LookAtBall&) override {}
  void reset(const LookAtGlobalBall&) override {}
  void reset(const LookAtPoint&) override {}
  void reset(const LookForward&) override {}
  void reset(const LookLeftAndRight& p) override
  {
    lookLeftAndRightSign = p.startLeft ? 1.f : -1.f;
  }
  void reset(const LookAtBallAndTarget& p) override
  {
    nextTurnedAngle = p.startTarget ? p.ballPositionAngle : Angle(p.walkingDirection.angle());
    lookAtBallFirst = !p.startTarget;
    lastHeadYawAngle = theJointAngles.angles[Joints::headYaw];
    lastTargetSwitch = 0;
    lookingAtBallSince = theFrameInfo.time;
    lastJointAnglesTime = theJointAngles.timestamp;
  }

  void setPanTiltRequest(HeadMotionRequest::CameraControlMode camera, Angle pan, Angle tilt, Angle speed, bool stopAndGoMode = false, bool calibrationMode = false)
  {
    theHeadMotionRequest.mode = calibrationMode ? HeadMotionRequest::calibrationMode : HeadMotionRequest::panTiltMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.pan = pan;
    theHeadMotionRequest.tilt = tilt;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
    theLibCheck.inc(LibCheck::headMotionRequest);
  }

  void setTargetOnGroundRequest(HeadMotionRequest::CameraControlMode camera, const Vector3f& target, Angle speed)
  {
    theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.target = target;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = false;
    theLibCheck.inc(LibCheck::headMotionRequest);
  }

  float lookLeftAndRightSign; /**< The side to which LookLeftAndRight currently turns the head. */
  bool lookAtBallFirst; /**< If true, then look at the ball first. */
  Angle nextTurnedAngle; /**< the last angle the head in LookAtBallTarget was turned */
  unsigned int lastTargetSwitch = 0;
  Angle lastHeadYawAngle = 0_deg;
  unsigned int lookingAtBallSince = 0;
  unsigned int lastJointAnglesTime = 0;
};

MAKE_SKILL_IMPLEMENTATION(HeadControlImpl);
