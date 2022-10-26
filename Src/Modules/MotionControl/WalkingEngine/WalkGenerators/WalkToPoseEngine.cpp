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
#include "Math/BHMath.h"

MAKE_MODULE(WalkToPoseEngine, motionControl);

void WalkToPoseEngine::update(WalkToPoseGenerator& walkToPoseGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:WalkToPoseEngine:betterPath");

  walkToPoseGenerator.createPhaseToTarget = [this](const Pose2f& targetInSCS, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                                   const Pose2f& walkSpeed, bool isLeftPhase, const MotionPhase& lastPhase,
                                                   const bool isFastWalkAllowed, const std::optional<Vector2f>& targetOfInterest, const bool sideWalkAllowed)
  {
    // doNotForceDiagonalWalk = true, because createPhaseToTarget is only used by the WalkToBallEngine.
    // Here we are walking towards the ball (targetOfInterest),
    // therefore use normal constraints to start side/diagonal walking
    return createPhase(targetInSCS, obstacleAvoidanceInSCS, walkSpeed, isLeftPhase, false, lastPhase, isFastWalkAllowed, targetOfInterest, sideWalkAllowed, false, true);
  };

  walkToPoseGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(motionRequest.walkTarget.translation.y() != 0.f ? (motionRequest.walkTarget.translation.y() > 0.f) : (motionRequest.walkTarget.rotation > 0.f), lastPhase);
    const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryDataPreview.inverse() * motionRequest.odometryData;
    const Pose2f targetInSCS = scsCognition * motionRequest.walkTarget;

    std::optional<Vector2f> targetOfInterest;
    if(motionRequest.targetOfInterest)
      targetOfInterest = scsCognition * motionRequest.targetOfInterest.value();
    MotionRequest::ObstacleAvoidance obstacleAvoidanceInSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceInSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceInSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    // doNotForceDiagonalWalk = false, so the robots always orientate towards the targetOfInterest
    return createPhase(targetInSCS, obstacleAvoidanceInSCS, motionRequest.walkSpeed, isLeftPhase,
                       motionRequest.keepTargetRotation, lastPhase, false, targetOfInterest, true, motionRequest.forceSideWalking, false);
  };
}

std::unique_ptr<MotionPhase> WalkToPoseEngine::createPhase(const Pose2f& targetInSCS,
                                                           const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                                           const Pose2f& walkSpeed, bool isLeftPhase, bool keepTargetRotation,
                                                           const MotionPhase& lastPhase, const bool isFastWalkAllowed,
                                                           const std::optional<Vector2f>& targetOfInterest, const bool sideWalkAllowed,
                                                           const bool forceSideWalk, const bool doNotForceDiagonalWalk) const
{
  Pose2f modTargetInSCS = targetInSCS;

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
    }
    if(toPoint.squaredNorm() <= sqr(startTurningBeforeCircleDistance))
    {
      const Vector2f offset = toPoint - segment.obstacle.center;
      const Pose2f controlPose(Angle::normalize(offset.angle() + (segment.clockwise ? -pi_2 - controlAheadAngle : pi_2 + controlAheadAngle)),
                               Pose2f(segment.clockwise ? -controlAheadAngle : controlAheadAngle, segment.obstacle.center) * (offset / std::cos(controlAheadAngle)));

      modTargetInSCS = controlPose;
    }
    else
      modTargetInSCS.translation = toPoint;
  }

  if(!keepTargetRotation)
  {
    bool isSideWalking = false;
    if(targetInSCS.translation.norm() > 100.f)
    {
      // Determine, if we should side walk to walk straight
      if(!(!theLibDemo.isOneVsOneDemoActive && theLibDemo.isDemoActive) && (sideWalkAllowed || forceSideWalk))
      {
        const Angle targetInSCSAngle = targetInSCS.translation.angle();
        const Angle sideOne = Angle::normalize(90_deg - targetInSCSAngle);
        const Angle sideTwo = Angle::normalize(-90_deg - targetInSCSAngle);
        Angle angleRefToSide = std::abs(sideOne) < std::abs(sideTwo) ? -sideOne : -sideTwo;

        Vector2f targetFocus;
        if(targetOfInterest)
          targetFocus = *targetOfInterest;
        else
          targetFocus = ((targetInSCS + Pose2f(-angleRefToSide, 0.f, 0.f)) + Vector2f(190.f, 0.f)).translation;
        const Angle targetFocusAngle = targetFocus.angle() - angleRefToSide;
        const Angle ballInVisionCorrection = targetFocusAngle - maxTargetFocusAngle.limit(targetFocusAngle);
        angleRefToSide += ballInVisionCorrection;
        const Angle orientationThreshold = (90_deg - std::abs(ballInVisionCorrection)) * 0.5f;
        if(targetOfInterest && !doNotForceDiagonalWalk)
        {
          const Angle targetFocusInModTarget = Angle::normalize(targetFocus.angle() - targetInSCSAngle);
          modTargetInSCS.rotation = Angle::normalize(targetInSCSAngle + targetFocusInModTarget - maxTargetFocusAngle.limit(targetFocusInModTarget));
        }
        else
          modTargetInSCS.rotation = targetInSCSAngle;
        const bool targetInFront = targetFocus.rotated(-(angleRefToSide - ballInVisionCorrection)).x() > 0.f;
        const bool preferSideWalkOverFront = std::abs(angleRefToSide) < orientationThreshold;
        const Rangea rangeAroundTarget(maxTargetFocusAngle.min + targetFocus.angle(), maxTargetFocusAngle.max + targetFocus.angle());
        const bool targetInSightAfterRot = rangeAroundTarget.isInside(modTargetInSCS.rotation) ||
                                           (rangeAroundTarget.min < -180_deg && Angle::normalize(rangeAroundTarget.min) < modTargetInSCS.rotation) ||
                                           (rangeAroundTarget.max > 180_deg && Angle::normalize(rangeAroundTarget.max) > modTargetInSCS.rotation);
        if(forceSideWalk || (targetOfInterest && !targetInSightAfterRot && !doNotForceDiagonalWalk))
        {
          modTargetInSCS.rotation = Angle::normalize(angleRefToSide + (targetInFront || targetOfInterest.has_value() ? 0_deg : 180_deg));
          isSideWalking = true;
        }
        else if(preferSideWalkOverFront &&
                targetInSCS.translation.squaredNorm() < sqr(1000.f) && // only if the target is under 1 meter away
                targetInFront)
        {
          modTargetInSCS.rotation = angleRefToSide;
          isSideWalking = true;
        }
      }
      else
        modTargetInSCS.rotation = targetInSCS.translation.angle();
      // Interpolate between the orientation walking to the target and the target orientation
      if(!isSideWalking)
      {
        Rangea forwardClip(-10_deg, 10_deg);
        if(forwardClip.isInside(modTargetInSCS.rotation))
          modTargetInSCS.rotation = 0_deg;
      }
      const float factor = mapToRange(targetInSCS.translation.norm(), 100.f, 300.f, 0.f, 1.f);
      modTargetInSCS.rotation = modTargetInSCS.rotation * factor + (1.f - factor) * targetInSCS.rotation;
    }
  }

  Pose2f step;
  step.rotation = theWalkGenerator.getRotationRange(isLeftPhase, walkSpeed).clamped((keepTargetRotation ? targetInSCS.rotation : modTargetInSCS.rotation));
  // In a rare case, the robot might switch the rotation direction with every step and therefore gets stuck.
  // This can be prevented by not allowing a rotation step.
  // It happens when the robot made a turn step, but now wants to hold the current orientation
  const Angle rotationWhenStopping = isLeftPhase ? theRobotModel.soleRight.rotation.getZAngle() : theRobotModel.soleLeft.rotation.getZAngle();
  // arbitrary values, difference should be small, the rotation itself large.
  // TODO evaluate if the 5_deg and 10_deg should get interpolated to a very low value based on the distance to the target
  if(std::abs(rotationWhenStopping + step.rotation / 2.f) < 5_deg && std::abs(step.rotation) > 10_deg)
    step.rotation = 0_deg;
  {
    std::vector<Vector2f> translationPolygon;
    std::vector<Vector2f> translationPolygonNoCenter;
    theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, walkSpeed, translationPolygon, translationPolygonNoCenter, modTargetInSCS.translation.norm() < 400.f && isFastWalkAllowed);
    if(Geometry::isPointInsideConvexPolygon(translationPolygonNoCenter.data(), static_cast<int>(translationPolygonNoCenter.size()), targetInSCS.translation))
      step.translation = targetInSCS.translation;
    else
    {
      Vector2f p1;
      const Vector2f targetDirectionWithAvoidance = obstacleAvoidanceInSCS.avoidance + modTargetInSCS.translation.normalized(std::max(0.f, 1.f - obstacleAvoidanceInSCS.avoidance.norm()));
      if(Geometry::isPointInsideConvexPolygon(translationPolygonNoCenter.data(), static_cast<int>(translationPolygonNoCenter.size()), Vector2f(0.f, 0.f)))
      {
        VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygonNoCenter, Geometry::Line(Vector2f(0.f, 0.f),
                                                               targetDirectionWithAvoidance / targetDirectionWithAvoidance.norm()), p1));
        step.translation = p1;
      }
      else
      {
        VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f),
                                                               targetDirectionWithAvoidance / targetDirectionWithAvoidance.norm()), p1));
        step.translation = p1;
        if(translationPolygonNoCenter[0].x() < 0.f)
          step.translation.x() = translationPolygonNoCenter[0].x();
        else
          step.translation.x() = translationPolygonNoCenter.back().x();
      }
    }
    if(isLeftPhase == (step.translation.y() < 0.f))
      step.translation.y() = 0.f;
  }
  const float useReferenceMaxForwardStep = useReferenceMaxForwardSpeed * theWalkingEngineOutput.walkStepDuration;
  // Do not stop in one step, but slow down. This results in a more stable walk
  if(step.translation.x() / useReferenceMaxForwardStep > reduceForwardStepUpperThreshold &&
     (targetInSCS.translation - step.translation).x() / useReferenceMaxForwardStep < reduceForwardStepLowerThreshold)
  {
    step.translation.x() *= reduceForwardStepUpperThreshold;
  }

  return theWalkGenerator.createPhase(step, lastPhase);
}
