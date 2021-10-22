/**
 * @file WalkToBallEngine.cpp
 *
 * This file implements a module that provides a walk to ball engine.
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#include "WalkToBallEngine.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(WalkToBallEngine, motionControl);

void WalkToBallEngine::update(WalkToBallGenerator& walkToBallGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:WalkToBallEngine:betterPath");
  walkToBallGenerator.createPhase = [this](const Pose2f& targetInSCS, const Vector2f& ballInSCS, const float distanceToBall, const MotionRequest::ObstacleAvoidance& obstacleAvoidanceInSCS,
                                           const Pose2f& walkSpeed, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false /* TODO */, lastPhase);

    if(!obstacleAvoidanceInSCS.path.empty())
      return theWalkToPoseGenerator.createPhaseToTarget(targetInSCS, obstacleAvoidanceInSCS, walkSpeed, isLeftPhase, lastPhase, false);

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

    if((std::abs(Angle::normalize((ballInSCS - targetInSCS.translation).angle() - targetInSCS.translation.angle())) < 90_deg ||
        abs(Angle::normalize((-ballInSCS).angle() - (targetInSCS.translation - ballInSCS).angle())) < 45_deg) && intersections == 0)
    {
      // case 1: no tangents necessary, walk directly to kick pose
      return theWalkToPoseGenerator.createPhaseToTarget(targetInSCS, obstacleAvoidanceInSCS, walkSpeed, isLeftPhase, lastPhase, true);
    }
    else
    {
      // 2 * tan(0.5 * angleDiff = 15deg/35deg/rotation) * ballCircleRadius = distanceToPoint
      const float tangentOffset = std::min(Constants::pi_2, Constants::pi_2 * ballCircleRadius / std::max(std::abs(ballInSCS.x()), std::abs(ballInSCS.y())));
      // The max(0.1f, ...) is necessary because otherwise tangentPoint1 and tangentPoint2 will be identical if the robot is on/inside the circle.
      const float cosTangentOffset = std::max(0.1f, std::cos(tangentOffset));

      const Vector2f tangentPoint1 = ballInSCS.rotated(tangentOffset) * cosTangentOffset;
      const Vector2f tangentPoint2 = ballInSCS.rotated(-tangentOffset) * cosTangentOffset;

      modTargetInSCS.translation = (tangentPoint1 - targetInSCS.translation).squaredNorm() < (tangentPoint2 - targetInSCS.translation).squaredNorm() ? tangentPoint1 : tangentPoint2;
      // find out which rotation for walking sideways makes the most sense and walk sideways, if the needed adjustment in the rotation is small enough (45deg)
      const Angle sideOne = Angle::normalize(90_deg - modTargetInSCS.translation.angle());
      const Angle sideTwo = Angle::normalize(-90_deg - modTargetInSCS.translation.angle());
      const Angle& angleRefToSide = std::abs(sideOne) < std::abs(sideTwo) ? -sideOne : -sideTwo;
      if(modTargetInSCS.translation.squaredNorm() > sqr(300.f))
      {
        if(std::abs(angleRefToSide) < 45_deg && targetInSCS.translation.squaredNorm() < sqr(1000.f)) // only if the target is under 1 meter away
          modTargetInSCS.rotation = angleRefToSide;
        else
          modTargetInSCS.rotation = modTargetInSCS.translation.angle();
      }
      else
      {
        const Angle angleOffset = (targetInSCS.inverse() * ballInSCS).angle(); // The angle that the ball has relative to the robot at the kick pose.
        if(std::abs(angleRefToSide) < 45_deg)
        {
          const float factor = std::min(1.f, std::max(0.f, modTargetInSCS.translation.norm()) / 100.f);
          modTargetInSCS.rotation = (1.f - factor) * Angle::normalize((ballInSCS - modTargetInSCS.translation.normalized(std::min(100.f, targetInSCS.translation.norm()))).angle() - angleOffset)
                                    + factor * angleRefToSide;
        }
        else
          modTargetInSCS.rotation = Angle::normalize((ballInSCS - modTargetInSCS.translation.normalized(std::min(100.f, targetInSCS.translation.norm()))).angle() - angleOffset);
      }

      // In a rare case, the robot might switch the rotation direction with every step and therefore gets stuck.
      // This can be prevented by not allowing a rotation step.
      const Angle rotationWhenStopping = isLeftPhase ? theRobotModel.soleRight.rotation.getZAngle() : theRobotModel.soleLeft.rotation.getZAngle();
      // arbitrary values, difference should be small, the rotation itself large.
      if(std::abs(rotationWhenStopping + modTargetInSCS.rotation / 2.f) < 5_deg && std::abs(modTargetInSCS.rotation) > 30_deg)
        modTargetInSCS.rotation = 0_deg;
    }

    Pose2f step;
    step.rotation = theWalkGenerator.getRotationRange(isLeftPhase, walkSpeed).clamped(modTargetInSCS.rotation);
    {
      std::vector<Vector2f> translationPolygon;
      theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, walkSpeed, translationPolygon, distanceToBall < 400.f);

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

    return theWalkGenerator.createPhase(step, lastPhase);
  };
}
