/**
 * @file WalkToBallEngine.cpp
 *
 * This file implements a module that provides a walk to ball engine.
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#include "WalkToBallEngine.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Settings.h"
#include "Math/Geometry.h"
#include "Tools/Motion/WalkUtilities.h"
#include "Tools/Motion/Transformation.h"

MAKE_MODULE(WalkToBallEngine);

WalkToBallEngine::WalkToBallEngine()
{
  ASSERT(lastLeftOverRotation.capacity() == lastExecutedStepRotation.capacity());
  for(std::size_t i = 0; i < lastExecutedStepRotation.capacity(); i++)
  {
    lastExecutedStepRotation.push_front(0_deg);
    lastLeftOverRotation.push_front(0_deg);
  }
  rotationReductionPerDirectionChange = 1.f / static_cast<float>(lastLeftOverRotation.capacity() + 1);
}

void WalkToBallEngine::update(WalkToBallGenerator& walkToBallGenerator)
{
  DECLARE_DEBUG_DRAWING3D("module:WalkToBallEngine:stepSize", "field");

  if(lastWalkStepUpdate != theWalkStepData.lastUpdate)
  {
    lastExecutedStepRotation.push_front(theWalkStepData.stepTarget.rotation);
    lastLeftOverRotation.push_front(tempLastLeftOverRotation);
    lastWalkStepUpdate = theWalkStepData.lastUpdate;
  }

  tempLastLeftOverRotation = !theWalkStepData.isLeftPhase ? theRobotModel.soleRight.rotation.getZAngle() : theRobotModel.soleLeft.rotation.getZAngle();

  walkToBallGenerator.createPhase = [this](const Pose2f& targetInSCS, const Vector2f& ballInSCS,
                                           const int timeSinceBallWasSeen, const Pose2f& scsCognition,
                                           const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                           const Pose2f& walkSpeed, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, targetInSCS);
    const bool fastWalk = timeSinceBallWasSeen < minTimeSinceBallSeen && ballInSCS.squaredNorm() < sqr(400.f);

    if(!obstacleAvoidanceInSCS.path.empty())
      return theWalkToPoseGenerator.createPhaseToTarget(targetInSCS, obstacleAvoidanceInSCS, walkSpeed,
                                                        isLeftPhase, lastPhase, scsCognition, fastWalk,
                                                        ballInSCS, timeSinceBallWasSeen < minTimeSinceBallSeen ? SideWalkingRequest::allowed : SideWalkingRequest::notAllowed);

    Pose2f modTargetInSCS = targetInSCS;
    const Angle ballAngle = ballInSCS.angle();
    if((std::abs(Angle::normalize((ballInSCS - targetInSCS.translation).angle() - targetInSCS.translation.angle())) < 90_deg ||
        std::abs(Angle::normalize(Angle::normalize(ballAngle - 180_deg) - (targetInSCS.translation - ballInSCS).angle())) < 45_deg))
    {
      // no tangents necessary, walk directly to kick pose
      return theWalkToPoseGenerator.createPhaseToTarget(targetInSCS, obstacleAvoidanceInSCS, walkSpeed,
                                                        isLeftPhase, lastPhase, scsCognition, fastWalk,
                                                        ballInSCS, timeSinceBallWasSeen < minTimeSinceBallSeen ? SideWalkingRequest::allowed : SideWalkingRequest::notAllowed);
    }
    else
    {
      const float ballCircleRadius = std::max((targetInSCS.translation - ballInSCS).norm(), theBallSpecification.radius + theRobotDimensions.soleToFrontEdgeLength + theWalkStepData.yHipOffset) +
                                     mapToRange(static_cast<float>(timeSinceBallWasSeen), 50.f, 200.f, 0.f, theBallSpecification.radius);
      Vector2f tangentPoint1;
      Vector2f tangentPoint2;
      const float ballDistance = ballInSCS.norm();
      if(ballDistance < ballCircleRadius)
      {
        // 2 * tan(0.5 * angleDiff = 15deg/35deg/rotation) * ballCircleRadius = distanceToPoint
        const float tangentOffset = std::min(Constants::pi_2, Constants::pi_2 * ballCircleRadius / std::max(std::abs(ballInSCS.x()), std::abs(ballInSCS.y())));
        // The max(0.1f, ...) is necessary because otherwise tangentPoint1 and tangentPoint2 will be identical if the robot is on/inside the circle.
        const float cosTangentOffset = std::max(0.1f, std::cos(tangentOffset));

        tangentPoint1 = ballInSCS.rotated(tangentOffset) * cosTangentOffset;
        tangentPoint2 = ballInSCS.rotated(-tangentOffset) * cosTangentOffset;
      }
      else
      {
        // I think this is based on the Thales's theorem, to determine the tangent point. This only works for ballInSCS.norm() > ballCircleRadius
        const float a = std::asin(ballCircleRadius / ballDistance);

        float t = ballAngle - a;
        tangentPoint1 = ballInSCS + Vector2f(ballCircleRadius * std::sin(t), ballCircleRadius * -std::cos(t));

        t = ballAngle + a;
        tangentPoint2 = ballInSCS + Vector2f(ballCircleRadius * -std::sin(t), ballCircleRadius * std::cos(t));
      }

      modTargetInSCS.translation = ((tangentPoint1 - targetInSCS.translation).norm() + tangentPoint1.norm()) < ((tangentPoint2 - targetInSCS.translation).norm() + tangentPoint2.norm()) ? tangentPoint1 : tangentPoint2;
      const bool useModTarget = targetInSCS.translation.x() * modTargetInSCS.translation.x() >= 0.f && targetInSCS.translation.y() * modTargetInSCS.translation.y() >= 0.f;

      if(targetInSCS.translation.squaredNorm() > sqr(1000.f) || timeSinceBallWasSeen > minTimeSinceBallSeen)
        WalkUtilities::calcDiagonal(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS, ballInSCS,
                                    isLeftPhase, fastWalk, lastPhase, true, useModTarget);
      else
      {
        Pose2f modTargetInSCS2 = modTargetInSCS;
        WalkUtilities::calcDiagonal(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS, ballInSCS,
                                    isLeftPhase, fastWalk, lastPhase, true, useModTarget);
        WalkUtilities::calcSideWalk(theWalkGenerator, theWalkingEngineOutput, walkSpeed, targetInSCS, modTargetInSCS2, ballInSCS,
                                    isLeftPhase, fastWalk, lastPhase, true);
        const float interpolationRange = theRobotDimensions.soleToFrontEdgeLength;
        const float interpolationOffset = theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + 200.f;
        const float factor = Rangef::ZeroOneRange().limit((ballDistance - interpolationOffset) / interpolationRange);
        if(std::abs(modTargetInSCS2.rotation * factor) <= std::abs(modTargetInSCS.rotation))
          modTargetInSCS.rotation = modTargetInSCS2.rotation;
      }

      // We are too close to the ball and will unintentionally touch the ball with the following step, when the y-translation is clip
      // Therefore it is better to walk backwards to gain some distance.
      // Overwrite after calculating the walk rotation. Otherwise WalkUtilities will calculate wrong angles!
      if(ballDistance < ballCircleRadius && isLeftPhase != (modTargetInSCS.translation.y() > 0.f))
        modTargetInSCS.translation = Vector2f(std::min(modTargetInSCS.translation.x(), ballDistance - ballCircleRadius), 0.f);
      else
      {
        // optimize step size to prevent small correction steps near the ball
        const Vector2f p1 = Geometry::getOrthogonalProjectionOfPointOnLine(Vector2f(0.f, 0.f), modTargetInSCS.translation.normalized(), targetInSCS.translation);
        const float distanceSqrMod = modTargetInSCS.translation.squaredNorm();
        const float distanceSqrP1 = p1.squaredNorm();
        if(distanceSqrP1 < sqr(400.f) && distanceSqrMod < sqr(300.f) && std::abs(p1.angle() - modTargetInSCS.translation.angle()) < 1_deg && p1.squaredNorm() > modTargetInSCS.translation.squaredNorm())
        {
          modTargetInSCS.translation.y() = p1.y();
          if(ballInSCS.x() * modTargetInSCS.translation.x() < 0.f)
            modTargetInSCS.translation.x() = p1.x();
        }
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
    }

    Pose2f step;
    step.rotation = theWalkGenerator.getRotationRange(isLeftPhase, walkSpeed).clamped(modTargetInSCS.rotation);
    {
      std::vector<Vector2f> translationPolygon;
      std::vector<Vector2f> translationPolygonNoCenter;
      theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, walkSpeed, translationPolygon, translationPolygonNoCenter, fastWalk, false);

      if(Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), modTargetInSCS.translation))
        step.translation = modTargetInSCS.translation;
      else
      {
        Vector2f p1;
        const Vector2f targetDirectionWithAvoidance = obstacleAvoidanceInSCS.avoidance + modTargetInSCS.translation.normalized(std::max(0.f, 1.f - obstacleAvoidanceInSCS.avoidance.norm()));
        if(Geometry::isPointInsideConvexPolygon(translationPolygonNoCenter.data(), static_cast<int>(translationPolygonNoCenter.size()), Vector2f(0.f, 0.f)))
        {
          VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygonNoCenter, Geometry::Line(Vector2f(0.f, 0.f),
                                                                 targetDirectionWithAvoidance / targetDirectionWithAvoidance.norm()), p1, false));
          step.translation = p1;
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
      if(Global::getSettings().robotType == Settings::nao && isLeftPhase == (step.translation.y() < 0.f))
        step.translation.y() = 0.f;
    }

    return theWalkGenerator.createPhase(step, lastPhase, 0.f);
  };
}
