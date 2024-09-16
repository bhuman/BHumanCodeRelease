/**
 * @file WalkToPoseEngine.cpp
 *
 * This file implements a module that provides a walk to pose generator.
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#include "WalkToPoseEngine.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/BHMath.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Motion/WalkUtilities.h"
#include "Tools/Motion/Transformation.h"

MAKE_MODULE(WalkToPoseEngine);

using namespace Motion::Transformation;

WalkToPoseEngine::WalkToPoseEngine()
{
  ASSERT(lastLeftOverRotation.capacity() == lastExecutedStepRotation.capacity());
  for(std::size_t i = 0; i < lastExecutedStepRotation.capacity(); i++)
  {
    lastExecutedStepRotation.push_front(0_deg);
    lastLeftOverRotation.push_front(0_deg);
  }
  rotationReductionPerDirectionChange = 1.f / static_cast<float>(lastLeftOverRotation.capacity() + 1);
}

void WalkToPoseEngine::update(WalkToPoseGenerator& walkToPoseGenerator)
{
  DECLARE_DEBUG_DRAWING3D("module:WalkToPoseEngine:stepSize", "field");

  if(lastWalkStepUpdate != theWalkStepData.lastUpdate)
  {
    lastExecutedStepRotation.push_front(theWalkStepData.stepTarget.rotation);
    lastLeftOverRotation.push_front(tempLastLeftOverRotation);
    lastWalkStepUpdate = theWalkStepData.lastUpdate;
  }

  tempLastLeftOverRotation = !theWalkStepData.isLeftPhase ? theRobotModel.soleRight.rotation.getZAngle() : theRobotModel.soleLeft.rotation.getZAngle();

  walkToPoseGenerator.createPhaseToTarget = [this](const Pose2f& targetInSCS, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                                   const Pose2f& walkSpeed, bool isLeftPhase, const MotionPhase& lastPhase, const Pose2f& scsCognition,
                                                   const bool isFastWalkAllowed, const std::optional<Vector2f>& targetOfInterest, const bool isSideWalkAllowed)
  {
    return createPhase(targetInSCS, scsCognition, obstacleAvoidanceInSCS, walkSpeed, isLeftPhase, false, lastPhase, isFastWalkAllowed, targetOfInterest, false, isSideWalkAllowed, false);
  };

  walkToPoseGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.walkTarget);
    const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theRobotDimensions, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);
    const Pose2f targetInSCS = scsCognition * motionRequest.walkTarget;

    std::optional<Vector2f> targetOfInterest;
    if(motionRequest.targetOfInterest)
      targetOfInterest = scsCognition * motionRequest.targetOfInterest.value();
    MotionRequest::ObstacleAvoidance obstacleAvoidanceInSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceInSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceInSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    // doNotForceDiagonalWalk = false, so the robots always orientate towards the targetOfInterest
    return createPhase(targetInSCS, scsCognition, obstacleAvoidanceInSCS, motionRequest.walkSpeed, isLeftPhase,
                       motionRequest.keepTargetRotation, lastPhase, false, targetOfInterest, motionRequest.forceSideWalking, true, true);
  };
}

std::unique_ptr<MotionPhase> WalkToPoseEngine::createPhase(const Pose2f& targetInSCS, const Pose2f&,
                                                           const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                                           const Pose2f& walkSpeed, bool isLeftPhase, bool keepTargetRotation,
                                                           const MotionPhase& lastPhase, const bool isFastWalkAllowed,
                                                           const std::optional<Vector2f>& targetOfInterest,
                                                           const bool forceSideWalk, const bool isSideWalkAllowed, const bool useModTarget) const
{
  Pose2f modTargetInSCS = targetInSCS;
  bool isObstacle = false;
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

    isObstacle = true;
    // We are not running into the obstacle anyway. No need to do an avoidance
    if(std::abs(targetInSCS.translation.angle()) > std::abs(modTargetInSCS.translation.angle()))
    {
      modTargetInSCS = targetInSCS;
      isObstacle = false;
    }
  }

  if(!keepTargetRotation)
  {
    if(targetInSCS.translation.norm() > 50.f)
    {
      const Vector2f targetFocus = targetOfInterest.has_value() ? targetOfInterest.value() : (targetInSCS + Vector2f(180.f, 0.f)).translation;

      // Determine, if we should side walk or walk straight
      if(!(!theLibDemo.isOneVsOneDemoActive && theLibDemo.isDemoActive))
      {
        if(forceSideWalk)
          WalkUtilities::calcSideWalk(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS, targetFocus,
                                      isLeftPhase, isFastWalkAllowed, lastPhase, useModTarget);
        else if(modTargetInSCS.translation.squaredNorm() > sqr(1000.f) || !isSideWalkAllowed)
          WalkUtilities::calcDiagonal(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS, targetFocus,
                                      isLeftPhase, isFastWalkAllowed, lastPhase);
        else
        {
          Pose2f modTargetInSCS2 = modTargetInSCS;
          WalkUtilities::calcSideWalk(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS, targetFocus,
                                      isLeftPhase, isFastWalkAllowed, lastPhase, useModTarget);
          WalkUtilities::calcDiagonal(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS2, targetFocus,
                                      isLeftPhase, isFastWalkAllowed, lastPhase);

          // When near the target, use the walk type (diagonal vs side walk) that will put the robot closer to the target pose
          const float factor = 1.f - Rangef::ZeroOneRange().limit((targetInSCS.translation.norm() - 200.f) / 100.f);
          const Angle targetRotFactor = factor * targetInSCS.rotation;
          if(std::abs(targetRotFactor - modTargetInSCS2.rotation) < std::abs(targetRotFactor - modTargetInSCS.rotation))
            modTargetInSCS = modTargetInSCS2;
        }
      }
      else
        modTargetInSCS.rotation = targetInSCS.translation.angle();

      const float factor = mapToRange(targetInSCS.translation.norm(), 50.f, 100.f, 0.f, 1.f);
      modTargetInSCS.rotation = modTargetInSCS.rotation * factor + (1.f - factor) * targetInSCS.rotation;
    }
    else
      modTargetInSCS.rotation = targetInSCS.rotation; // Obstacle avoidance shall not set the rotation
  }

  // In a rare case, the robot might switch the rotation direction with every step and therefore gets stuck.
  // This can be prevented by not allowing a rotation step.
  // It happens when the robot made a turn step, but now wants to hold the current orientation
  auto shouldReduceStepRotation = [&](const Angle leftOverRotation, const Angle stepRotation)
  {
    // arbitrary values, difference should be small, the rotation itself large.
    return std::abs(leftOverRotation + stepRotation / 2.f) < 5_deg && std::abs(stepRotation) > 10_deg;
  };

  float stepRotationReduction = 0.f;
  for(std::size_t i = 0; i < lastExecutedStepRotation.capacity(); i++)
  {
    if(shouldReduceStepRotation(lastLeftOverRotation[i], lastExecutedStepRotation[i]))
      stepRotationReduction += rotationReductionPerDirectionChange;
    else
      break;
  }

  const Angle rotationWhenStopping = isLeftPhase ? theRobotModel.soleRight.rotation.getZAngle() : theRobotModel.soleLeft.rotation.getZAngle();
  if(shouldReduceStepRotation(rotationWhenStopping, modTargetInSCS.rotation))
    stepRotationReduction += rotationReductionPerDirectionChange;
  else
    stepRotationReduction = 0.f;
  modTargetInSCS.rotation *= Rangef::ZeroOneRange().limit((1.f - stepRotationReduction));

  modTargetInSCS.rotation = theWalkGenerator.getRotationRange(isLeftPhase, walkSpeed).clamped((keepTargetRotation ? targetInSCS.rotation : modTargetInSCS.rotation));

  const bool isFastWalk = modTargetInSCS.translation.norm() < 400.f && isFastWalkAllowed;
  Pose2f step = generateStep(isLeftPhase, lastPhase, walkSpeed, isFastWalk, targetInSCS, modTargetInSCS, obstacleAvoidanceInSCS, isObstacle);
  if(!isFastWalk && step.translation.y() != 0.f && isObstacle)
  {
    const Pose2f otherStep = generateStep(isLeftPhase, lastPhase, walkSpeed, true, targetInSCS, modTargetInSCS, obstacleAvoidanceInSCS, isObstacle);
    step.translation.y() = otherStep.translation.y();
  }

  return theWalkGenerator.createPhase(step, lastPhase, 0.f);
}

Pose2f WalkToPoseEngine::generateStep(const bool isLeftPhase, const MotionPhase& lastPhase, const Pose2f& walkSpeed,
                                      const bool isFastWalk, const Pose2f& targetInSCS, const Pose2f& modTargetInSCS,
                                      const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS, const bool isObstacle) const
{
  Pose2f step = modTargetInSCS;
  std::vector<Vector2f> translationPolygon;
  std::vector<Vector2f> translationPolygonNoCenter;
  theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, walkSpeed, translationPolygon, translationPolygonNoCenter, isFastWalk, false);
  if(Geometry::isPointInsideConvexPolygon(translationPolygonNoCenter.data(), static_cast<int>(translationPolygonNoCenter.size()), targetInSCS.translation))
    step.translation = targetInSCS.translation;
  else
  {
    Vector2f p1;
    const Vector2f targetDirectionWithAvoidance = obstacleAvoidanceInSCS.avoidance + modTargetInSCS.translation.normalized(std::max(0.f, 1.f - obstacleAvoidanceInSCS.avoidance.norm()));
    if(Geometry::isPointInsideConvexPolygon(translationPolygonNoCenter.data(), static_cast<int>(translationPolygonNoCenter.size()), Vector2f(0.f, 0.f)))
    {
      VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygonNoCenter, Geometry::Line(Vector2f(0.f, 0.f),
                                                             targetDirectionWithAvoidance / targetDirectionWithAvoidance.norm()), p1, false));
      step.translation = p1;

      if(isLeftPhase != (p1.y() < 0.f) && isObstacle &&
         ((step.translation.x() > 0.f && translationPolygonNoCenter[1].x() / (theWalkingEngineOutput.maxSpeed.translation.x() * theWalkingEngineOutput.stepDuration / 1000.f) < 0.5f) ||
          (step.translation.x() < 0.f && translationPolygonNoCenter[translationPolygonNoCenter.size() - 2].x() / (theWalkingEngineOutput.maxBackwardStepSize * theWalkingEngineOutput.stepDuration / 1000.f) < 0.75f)))
      {
        if(std::abs(modTargetInSCS.translation.y()) >= std::abs(p1.y()))
        {
          const float sign = static_cast<float>(sgnPos(modTargetInSCS.translation.y()));
          Vector2f newTarget(modTargetInSCS.translation.x(), std::min(std::abs(modTargetInSCS.translation.y()), translationPolygonNoCenter[0].y()) * sign);
          Vector2f p2;
          const Vector2f newDirection(targetDirectionWithAvoidance.x(), 0.f);
          VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygonNoCenter, Geometry::Line(Vector2f(0.f, newTarget.y() * 0.99f),
                                                                 newDirection / newDirection.norm()), p2, false));
          step.translation.y() = p2.y();
        }
      }
    }
    else
    {
      VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f),
                                                             targetDirectionWithAvoidance / targetDirectionWithAvoidance.norm()), p1, false));
      step.translation = p1;
      if(translationPolygonNoCenter[0].x() < 0.f)
        step.translation.x() = translationPolygonNoCenter[0].x();
      else
        step.translation.x() = translationPolygonNoCenter.back().x();
    }
  }
  if(isLeftPhase == (step.translation.y() < 0.f))
    step.translation.y() = 0.f;

  const float useReferenceMaxForwardStep = useReferenceMaxForwardSpeed * theWalkingEngineOutput.walkStepDuration;
  // Do not stop in one step, but slow down. This results in a more stable walk
  if(step.translation.x() / useReferenceMaxForwardStep > reduceForwardStepUpperThreshold &&
     (targetInSCS.translation - step.translation).x() / useReferenceMaxForwardStep < reduceForwardStepLowerThreshold)
  {
    step.translation.x() *= reduceForwardStepUpperThreshold;
  }
  return step;
}
