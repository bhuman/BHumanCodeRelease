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
#include "Tools/Motion/WalkUtilities.h"
#include "Tools/Motion/Transformation.h"

MAKE_MODULE(WalkToPoseEngine);

using namespace Motion::Transformation;

void WalkToPoseEngine::update(WalkToPoseGenerator& walkToPoseGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:WalkToPoseEngine:betterPath");

  walkToPoseGenerator.createPhaseToTarget = [this](const Pose2f& targetInSCS, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                                   const Pose2f& walkSpeed, bool isLeftPhase, const MotionPhase& lastPhase,
                                                   const bool isFastWalkAllowed, const std::optional<Vector2f>& targetOfInterest, const bool isSideWalkAllowed)
  {
    return createPhase(targetInSCS, obstacleAvoidanceInSCS, walkSpeed, isLeftPhase, false, lastPhase, isFastWalkAllowed, targetOfInterest, false, isSideWalkAllowed, false);
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
    return createPhase(targetInSCS, obstacleAvoidanceInSCS, motionRequest.walkSpeed, isLeftPhase,
                       motionRequest.keepTargetRotation, lastPhase, false, targetOfInterest, motionRequest.forceSideWalking, true, true);
  };
}

std::unique_ptr<MotionPhase> WalkToPoseEngine::createPhase(const Pose2f& targetInSCS,
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
        else if(targetInSCS.translation.squaredNorm() > sqr(1000.f) || !isSideWalkAllowed)
          WalkUtilities::calcDiagonal(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS, targetFocus,
                                      isLeftPhase, isFastWalkAllowed, lastPhase);
        else
        {
          Pose2f modTargetInSCS2 = modTargetInSCS;
          WalkUtilities::calcSideWalk(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS, targetFocus,
                                      isLeftPhase, isFastWalkAllowed, lastPhase, useModTarget);
          WalkUtilities::calcDiagonal(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS2, targetFocus,
                                      isLeftPhase, isFastWalkAllowed, lastPhase);
          const float factor = Rangef::ZeroOneRange().limit((targetFocus.norm() - 350.f) / 100.f);
          if(std::abs(modTargetInSCS2.rotation) < std::abs(modTargetInSCS.rotation * factor))
            modTargetInSCS.rotation = modTargetInSCS2.rotation;
        }
      }
      else
        modTargetInSCS.rotation = targetInSCS.translation.angle();

      // If the target is currently in front of us but would be behind us if we take the target rotation, then we want to rotate as late as possible
      // If not, then we can start rotating to the target rotation from further away
      const Vector2f targetRotated = targetInSCS.translation.rotated(-targetInSCS.rotation);
      const float xInterpolationRange = Rangef::ZeroOneRange().limit(std::min(targetRotated.x() - targetInSCS.translation.x(), 0.f) / -35.f);

      const float factor = mapToRange(targetInSCS.translation.norm(), xInterpolationRange * 50.f + (1.f - xInterpolationRange) * 100.f, xInterpolationRange * 100.f + (1.f - xInterpolationRange) * 300.f, 0.f, 1.f);
      modTargetInSCS.rotation = modTargetInSCS.rotation * factor + (1.f - factor) * targetInSCS.rotation;
    }
    else
      modTargetInSCS.rotation = targetInSCS.rotation; // Obstacle avoidance shall not set the rotation
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
    theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, walkSpeed, translationPolygon, translationPolygonNoCenter, modTargetInSCS.translation.norm() < 400.f && isFastWalkAllowed, false);
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
                                                                   newDirection / newDirection.norm()), p2));
            step.translation.y() = p2.y();
          }
        }
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

  return theWalkGenerator.createPhase(step, lastPhase, 0.f);
}
