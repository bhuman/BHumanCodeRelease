/**
 * @file WalkToPoseEngine.cpp
 *
 * This file implements a module that provides a walk to pose generator.
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#include "WalkToPoseEngine.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Math/BHMath.h"

MAKE_MODULE(WalkToPoseEngine, motionControl);

void WalkToPoseEngine::update(WalkToPoseGenerator& walkToPoseGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:WalkToPoseEngine:betterPath");

  walkToPoseGenerator.createPhaseToTarget = [this](const Pose2f& targetInSCS, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS, const Pose2f& walkSpeed, bool isLeftPhase, const MotionPhase& lastPhase, const bool isFastWalkAllowed)
  {
    return createPhase(targetInSCS, obstacleAvoidanceInSCS, walkSpeed, isLeftPhase, false, lastPhase, isFastWalkAllowed);
  };

  walkToPoseGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(motionRequest.walkTarget.translation.y() != 0.f ? (motionRequest.walkTarget.translation.y() > 0.f) : (motionRequest.walkTarget.rotation > 0.f), lastPhase);
    const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryData.inverse() * motionRequest.odometryData;
    const Pose2f targetInSCS = scsCognition * motionRequest.walkTarget;

    MotionRequest::ObstacleAvoidance obstacleAvoidanceInSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceInSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceInSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    return createPhase(targetInSCS, obstacleAvoidanceInSCS, motionRequest.walkSpeed, isLeftPhase, motionRequest.keepTargetRotation, lastPhase, false);
  };
}

std::unique_ptr<MotionPhase> WalkToPoseEngine::createPhase(const Pose2f& targetInSCS, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS, const Pose2f& walkSpeed, bool isLeftPhase, bool keepTargetRotation, const MotionPhase& lastPhase, const bool isFastWalkAllowed) const
{
  Pose2f modTargetInSCS = targetInSCS;

  bool interpolateRotation = !keepTargetRotation;
  if(!obstacleAvoidanceInSCS.path.empty())
  {
    const auto& segment = obstacleAvoidanceInSCS.path.front();
    const float sign = segment.clockwise ? 1.f : -1.f;
    Vector2f toPoint = segment.obstacle.center;
    if(segment.obstacle.radius != 0.f)
    {
      const Angle tangentOffset = std::asin(std::min(1.f, segment.obstacle.radius / segment.obstacle.center.norm()));
      toPoint.rotate(sign * tangentOffset);
      toPoint *= std::cos(tangentOffset);
      interpolateRotation = false;
    }
    if(toPoint.squaredNorm() <= sqr(startTurningBeforeCircleDistance))
    {
      const Vector2f offset = toPoint - segment.obstacle.center;
      const Pose2f controlPose(Angle::normalize(offset.angle() + (segment.clockwise ? -pi_2 - controlAheadAngle : pi_2 + controlAheadAngle)),
                               Pose2f(segment.clockwise ? -controlAheadAngle : controlAheadAngle, segment.obstacle.center) * (offset / std::cos(controlAheadAngle)));

      modTargetInSCS = controlPose;
    }
    else
    {
      modTargetInSCS.translation = toPoint;
      modTargetInSCS.rotation = toPoint.angle();
    }
  }

  if(interpolateRotation)
  {
    // This threshold should depend on how different current rotation and target rotation are.
    // If target abs(rotation) low (I am already aligned) -> be allowed to walk up to, say, 60cm omnidirectionally (i.e. sidewards/diagonal/backwards)
    // Features: abs(targetInSCS.rotation), abs(normalize(targetInSCS.translation.angle() - targetInSCS.rotation)), abs(targetInSCS.translation.angle()), targetInSCS.norm()

    if((std::abs(Angle::normalize(targetInSCS.translation.angle() - targetInSCS.rotation)) < 90_deg ||
        targetInSCS.translation.x() > 0.f) &&
       targetInSCS.translation.norm() > 100.f)
    {
      Angle direction = targetInSCS.translation.angle();
      Rangea forwardClip(-10_deg, 10_deg);
      if(forwardClip.isInside(direction))
        direction = 0_deg;
      const float factor = Rangef::ZeroOneRange().limit((targetInSCS.translation.norm() - 500.f) / 500.f);
      const Angle directionNear = (factor * targetInSCS.translation.normalized() + Vector2f::polar(1.f - factor, targetInSCS.rotation)).angle();
      modTargetInSCS.rotation = factor * direction + (1.f - factor) * directionNear;
    }
    else if(targetInSCS.translation.norm() > 100.f)
    {
      if(targetInSCS.translation.norm() > 600.f - std::abs(targetInSCS.rotation) / pi * 400.f)
      {
        modTargetInSCS.rotation = targetInSCS.translation.angle();
      }
      else
      {
        const float factor = (targetInSCS.translation.norm() - 100.f) / 500.f;
        modTargetInSCS.rotation = (factor * targetInSCS.translation.normalized() + Vector2f::polar(1.f - factor, targetInSCS.rotation)).angle();
      }
    }
    const Angle sideOne = Angle::normalize(90_deg - modTargetInSCS.translation.angle());
    const Angle sideTwo = Angle::normalize(-90_deg - modTargetInSCS.translation.angle());
    const Angle& angleRefToSide = std::abs(sideOne) < std::abs(sideTwo) ? -sideOne : -sideTwo;
    if(modTargetInSCS.translation.squaredNorm() > sqr(300.f))
    {
      if(std::abs(angleRefToSide) < 45_deg && targetInSCS.translation.squaredNorm() < sqr(1000.f))   // only if the target is under 1 meter away
        modTargetInSCS.rotation = angleRefToSide;
      else
        modTargetInSCS.rotation = modTargetInSCS.translation.angle();
    }
    // In a rare case, the robot might switch the rotation direction with every step and therefore gets stuck.
    // This can be prevented by not allowing a rotation step.
    const Angle rotationWhenStopping = isLeftPhase ? theRobotModel.soleRight.rotation.getZAngle() : theRobotModel.soleLeft.rotation.getZAngle();
    // arbitrary values, difference should be small, the rotation itself large.
    if(std::abs(rotationWhenStopping + modTargetInSCS.rotation / 2.f) < 5_deg && std::abs(modTargetInSCS.rotation) > 30_deg)
      modTargetInSCS.rotation = 0_deg;
  }

  Pose2f step;
  step.rotation = theWalkGenerator.getRotationRange(isLeftPhase, walkSpeed).clamped(keepTargetRotation ? targetInSCS.rotation : modTargetInSCS.rotation);
  {
    std::vector<Vector2f> translationPolygon;
    theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, walkSpeed, translationPolygon, modTargetInSCS.translation.norm() < 400.f && isFastWalkAllowed);
    if(Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), targetInSCS.translation))
      step.translation = targetInSCS.translation;
    else
    {
      Vector2f p1;
      const Vector2f targetDirectionWithAvoidance = obstacleAvoidanceInSCS.avoidance + modTargetInSCS.translation.normalized(std::max(0.f, 1.f - obstacleAvoidanceInSCS.avoidance.norm()));
      VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f), targetDirectionWithAvoidance / targetDirectionWithAvoidance.norm()), p1));
      step.translation = p1;
    }
    if(isLeftPhase == (step.translation.y() < 0.f))
      step.translation.y() = 0.f;
  }

  // Do not stop in one step, but slow down. This results in a more stable walk
  if(step.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x() / theWalkingEngineOutput.walkStepDuration > reduceForwardStepUpperThreshold &&
     (targetInSCS.translation - step.translation).x() / theWalkingEngineOutput.maxSpeed.translation.x() / theWalkingEngineOutput.walkStepDuration < reduceForwardStepLowerThreshold)
  {
    // reduce forward step size to 75%
    const float factor = step.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x() / theWalkingEngineOutput.walkStepDuration;
    step.translation.x() *= reduceForwardStepUpperThreshold / factor;
  }

  return theWalkGenerator.createPhase(step, lastPhase);
}
