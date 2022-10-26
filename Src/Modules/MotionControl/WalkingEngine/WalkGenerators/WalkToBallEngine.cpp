/**
 * @file WalkToBallEngine.cpp
 *
 * This file implements a module that provides a walk to ball engine.
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#include "WalkToBallEngine.h"
#include "Math/Geometry.h"

MAKE_MODULE(WalkToBallEngine, motionControl);

// TODO disable side walking if ball was not seen for x seconds!
void WalkToBallEngine::update(WalkToBallGenerator& walkToBallGenerator)
{
  walkToBallGenerator.createPhase = [this](const Pose2f& targetInSCS, const Vector2f& ballInSCS,
                                           const int timeSinceBallWasSeen, const float distanceToBall,
                                           const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                           const Pose2f& walkSpeed, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false /* TODO */, lastPhase);

    if(!obstacleAvoidanceInSCS.path.empty())
      return theWalkToPoseGenerator.createPhaseToTarget(targetInSCS, obstacleAvoidanceInSCS, walkSpeed,
                                                        isLeftPhase, lastPhase, false,
                                                        ballInSCS, timeSinceBallWasSeen < minTimeSinceBallSeen);

    const float ballCircleRadius = std::max((targetInSCS.translation - ballInSCS).norm(), theBallSpecification.radius + 110.f + 50.f); // 110 = length of the foot in direction of the ball. Not using theRobotDimensions.footLength because this should be calculated. 50 = safety zone
    Pose2f modTargetInSCS = targetInSCS;

    int intersections = 0;
    // Only check ball collision when near the ball
    if(ballInSCS.squaredNorm() < sqr(300.f))
    {
      // Based on the swing foot movement, check if the foot will collide with the ball. If so, do not use theWalkToPoseGenerator
      const FootValues& foot = isLeftPhase ? theFootOffset.leftFoot : theFootOffset.rightFoot;
      Vector2f shift(0.f, 0.f);
      shift.x() = isLeftPhase ? -theRobotModel.soleLeft.translation.x() + theRobotModel.soleRight.translation.x() : theRobotModel.soleLeft.translation.x() - theRobotModel.soleRight.translation.x();
      Vector2f intersection1, intersection2;
      const Vector2f startPoint = shift + Vector2f(theFootOffset.forward, isLeftPhase ? foot.left : -foot.right);
      const Vector2f endPoint = targetInSCS * Vector2f(theFootOffset.forward, isLeftPhase ? foot.left : -foot.right);
      int intersections = Geometry::getIntersectionOfLineAndCircle(Geometry::Line(Vector2f(startPoint), (endPoint - startPoint).normalized()),
                          Geometry::Circle(ballInSCS + shift, theBallSpecification.radius + 30.f), intersection1, intersection2);

      // Based on the direction, we know if the collision is between the foot points or outside
      const Angle pointDirection = intersections == 0 ? 0.f : (startPoint - endPoint).angle();
      const Angle intersection1Angle = intersections == 0 ? 0.f : (intersection1 - endPoint).angle();
      const Angle intersection2Angle = intersections == 0 ? 0.f : (intersection2 - endPoint).angle();

      // 0.01deg as a small floating point inaccuracy
      if(std::abs(intersection1Angle - pointDirection) > 0.01_deg && std::abs(intersection2Angle - pointDirection) > 0.01_deg)
        intersections = 0;
    }

    const Angle ballAngle = ballInSCS.angle();
    if((std::abs(Angle::normalize((ballInSCS - targetInSCS.translation).angle() - targetInSCS.translation.angle())) < 90_deg ||
        abs(Angle::normalize(Angle::normalize(ballAngle - 180_deg) - (targetInSCS.translation - ballInSCS).angle())) < 45_deg) && intersections == 0)
    {
      // case 1: no tangents necessary, walk directly to kick pose
      return theWalkToPoseGenerator.createPhaseToTarget(targetInSCS, obstacleAvoidanceInSCS, walkSpeed, isLeftPhase,
                                                        lastPhase, timeSinceBallWasSeen < minTimeSinceBallSeen,
                                                        ballInSCS, timeSinceBallWasSeen < minTimeSinceBallSeen);
    }
    else
    {
      Vector2f tangentPoint1;
      Vector2f tangentPoint2;
      if(ballInSCS.norm() < ballCircleRadius)
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
        // I think this is based on the Thales's theorem, to determin the tangent point. This only works for ballInSCS.norm() < ballCircleRadius
        const float a = std::asin(ballCircleRadius / ballInSCS.norm());

        float t = ballAngle - a;
        tangentPoint1 = ballInSCS + Vector2f(ballCircleRadius * std::sin(t), ballCircleRadius * -std::cos(t));

        t = ballAngle + a;
        tangentPoint2 = ballInSCS + Vector2f(ballCircleRadius * -std::sin(t), ballCircleRadius * std::cos(t));
      }

      modTargetInSCS.translation = (tangentPoint1 - targetInSCS.translation).squaredNorm() < (tangentPoint2 - targetInSCS.translation).squaredNorm() ? tangentPoint1 : tangentPoint2;
      // find out which rotation for walking sideways makes the most sense and walk sideways, if the needed adjustment in the rotation is small enough (45deg)
      Angle angleRefToSide = modTargetInSCS.translation.angle();
      Angle orientationThreshold = 0_deg;
      Angle ballInVisionCorrection = 0_deg;
      if(timeSinceBallWasSeen < minTimeSinceBallSeen)
      {
        const Angle sideOne = Angle::normalize(90_deg - modTargetInSCS.translation.angle());
        const Angle sideTwo = Angle::normalize(-90_deg - modTargetInSCS.translation.angle());
        angleRefToSide = std::abs(sideOne) < std::abs(sideTwo) ? -sideOne : -sideTwo;

        ballInVisionCorrection = ballAngle - maxTargetFocusAngle.limit(ballAngle);
        angleRefToSide += ballInVisionCorrection;
        orientationThreshold = (90_deg - std::abs(ballInVisionCorrection)) * 0.5f;
      }
      if(modTargetInSCS.translation.squaredNorm() > sqr(300.f))
      {
        if(std::abs(angleRefToSide) < orientationThreshold && targetInSCS.translation.squaredNorm() < sqr(1000.f) && ballInSCS.rotated(-(angleRefToSide - ballInVisionCorrection)).x() > 0.f)  // only if the target is under 1 meter away
          modTargetInSCS.rotation = angleRefToSide;
        else
          modTargetInSCS.rotation = modTargetInSCS.translation.angle();
      }
      else
      {
        const Angle angleOffset = (targetInSCS.inverse() * ballInSCS).angle(); // The angle that the ball has relative to the robot at the kick pose.
        if(std::abs(angleRefToSide) < orientationThreshold && ballInSCS.rotated(-angleRefToSide).x() > 0.f)
        {
          const float factor = Rangef::ZeroOneRange().limit(modTargetInSCS.translation.norm() / 100.f);
          modTargetInSCS.rotation = (1.f - factor) * Angle::normalize((ballInSCS - modTargetInSCS.translation.normalized(std::min(100.f, targetInSCS.translation.norm()))).angle() - angleOffset)
                                    + factor * angleRefToSide;
        }
        else
          modTargetInSCS.rotation = Angle::normalize((ballInSCS - modTargetInSCS.translation.normalized(std::min(100.f, targetInSCS.translation.norm()))).angle() - angleOffset);
      }

      // In a rare case, the robot might switch the rotation direction with every step and therefore gets stuck.
      // This can be prevented by not allowing a rotation step.
      // It happens when the robot made a turn step, but now wants to hold the current orientation
      const Angle rotationWhenStopping = isLeftPhase ? theRobotModel.soleRight.rotation.getZAngle() : theRobotModel.soleLeft.rotation.getZAngle();
      // arbitrary values, difference should be small, the rotation itself large.
      if(std::abs(rotationWhenStopping + modTargetInSCS.rotation / 2.f) < 5_deg && std::abs(modTargetInSCS.rotation) > 10_deg)
        modTargetInSCS.rotation = 0_deg;
    }

    Pose2f step;
    step.rotation = theWalkGenerator.getRotationRange(isLeftPhase, walkSpeed).clamped(modTargetInSCS.rotation);
    {
      std::vector<Vector2f> translationPolygon;
      std::vector<Vector2f> translationPolygonNoCenter;
      theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, walkSpeed, translationPolygon, translationPolygonNoCenter, (timeSinceBallWasSeen < minTimeSinceBallSeen) && distanceToBall < 400.f);

      if(Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), targetInSCS.translation))
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
      }
      if(isLeftPhase == (step.translation.y() < 0.f))
        step.translation.y() = 0.f;
    }

    return theWalkGenerator.createPhase(step, lastPhase);
  };
}
