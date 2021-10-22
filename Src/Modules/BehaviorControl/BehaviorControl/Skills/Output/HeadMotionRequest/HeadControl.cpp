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
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include <cmath>

SKILL_IMPLEMENTATION(HeadControl,
{,
  IMPLEMENTS(LookActive),
  IMPLEMENTS(LookAtAngles),
  IMPLEMENTS(LookAtBall),
  IMPLEMENTS(LookAtGlobalBall),
  IMPLEMENTS(LookAtPoint),
  IMPLEMENTS(LookForward),
  IMPLEMENTS(LookLeftAndRight),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(HeadMotionInfo),
  REQUIRES(JointAngles),
  REQUIRES(LibCheck),
  REQUIRES(LibLookActive),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  MODIFIES(HeadMotionRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) ownBallTimeout, /**< LookAtBall will use the team ball or look active if the ball hasn't been seen for this time. */
    (int)(500) ownBallDisappearedTimeout, /**< LookAtBall will use the team ball or look active if the ball disappeared for this time. */
  }),
});

class HeadControl : public HeadControlBase
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
    if(useOwnEstimate || theTeamBallModel.isValid)
    {
      const Vector2f ballPosition = useOwnEstimate ?
                                    (p.mirrored ? (theRobotPose.inversePose * (theRobotPose * theBallModel.estimate.position).rotated(pi)) : theBallModel.estimate.position) :
                                    (theRobotPose.inversePose * (p.mirrored ? theTeamBallModel.position.rotated(pi) : theTeamBallModel.position));
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
    if(theTeamBallModel.isValid)
    {
      const Vector2f ballPosition = theRobotPose.inversePose * (p.mirrored ? theTeamBallModel.position.rotated(pi) : theTeamBallModel.position);
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
};

MAKE_SKILL_IMPLEMENTATION(HeadControl);
