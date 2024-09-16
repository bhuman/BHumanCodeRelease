/**
 * @file WalkPotentialField.cpp
 *
 * This file implements an implementation for the WalkPotentialField skill.
 *
 * @author Andreas Stolpmann
 * @author somebody unknown
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 * @author Sina Schreiber
 * @author Fynn Böse
 */

#include "SkillBehaviorControl.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Debugging.h"
#include "Math/BHMath.h"
#include "Math/Boundary.h"
#include "Math/Geometry.h"
#include "Framework/Settings.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include <cmath>

option((SkillBehaviorControl) WalkPotentialField,
       args((const Vector2f&) target,
            (bool) straight,
            (float) ballFactor,
            (bool) useRotation,
            (float) rotation,
            (const std::optional<Vector2f>&) targetOfInterest),
       defs((float)(250.f) alignDistance, /**< If the target is closer than this distance, the robot is completely aligned to the ball or the target rotation. */
            (Angle)(45_deg) maxBallAngleOffset, /**< If not walking straight, the ball should have at most this angle relative to the robot's orientation. */
            (float)(600.f) maxRejection,
            (float)(400.f) maxDistance,
            (float)(750.f) freeKickClearAreaRadius, /**< The radius of the area that has to be cleared by the defending team during a free kick. */
            (float)(350.f) gridOffset, /**< Spacing for the origins of the arrows in the debug drawing of the potential field */
            (int)(15) arrowWidth)) /**< Pen width for the arrow lines in the debug drawing of the potential field */
{
  static_cast<void>(arrowWidth); // Unused in Release
  const Vector2f opponentGoalPosition(theFieldDimensions.xPosOpponentGoalLine, 0.f);
  const Vector2f ownGoalPosition(theFieldDimensions.xPosOwnGoalLine, 0.f);
  const Vector2f ownLeftGoalPost(theFieldDimensions.xPosOwnGoalPost + theFieldDimensions.fieldLinesWidth * 0.5f, theFieldDimensions.yPosLeftGoal - theBallSpecification.radius * 2.f);
  const Vector2f ownRightGoalPost(theFieldDimensions.xPosOwnGoalPost + theFieldDimensions.fieldLinesWidth * 0.5f, theFieldDimensions.yPosRightGoal + theBallSpecification.radius * 2.f);
  const Vector2f ownPenaltyAreaCenter(theFieldDimensions.xPosOwnGoalLine - 1000.0f, 0.0f);
  const Vector2f opponentPenaltyAreaCenter(theFieldDimensions.xPosOpponentGoalLine + 1000.0f, 0.0f);
  const Vector2f ownLeftPenaltyAreaT(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f ownRightPenaltyAreaT(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f opponentLeftPenaltyAreaT(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f opponentRightPenaltyAreaT(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f ownLeftPenaltyAreaL(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f ownRightPenaltyAreaL(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f opponentLeftPenaltyAreaL(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f opponentRightPenaltyAreaL(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  const float halfLineWidth = theFieldDimensions.fieldLinesWidth * 0.5f;
  const Boundaryf reducedRect(Rangef(theFieldDimensions.xPosOwnGoalLine - halfLineWidth + maxDistance, theFieldDimensions.xPosOpponentGoalLine + halfLineWidth - maxDistance),
                              Rangef(theFieldDimensions.yPosRightTouchline - halfLineWidth + maxDistance, theFieldDimensions.yPosLeftTouchline + halfLineWidth - maxDistance));

  const auto computeVectorPotentialFieldTarget = [&](const Vector2f& position, const Vector2f& target) -> Vector2f
  {
    // radius around the target where slowing down is started
    const float slowDownRadius = 500.0f;
    const float maxSpeed = 200.0f;
    // directional vector to target
    const Vector2f difference = target - position;
    // attraction is weighted with distance to target, but clipped with slow down radius
    const float length = std::min(maxSpeed, difference.norm() * maxSpeed / slowDownRadius);
    return difference.normalized(length);
  };

  const auto computeVectorPotentialFieldPenaltyArea = [&](const Vector2f& position, const bool ownPenaltyArea) -> Vector2f
  {
    const Vector2f direction = ownPenaltyArea ? position - ownPenaltyAreaCenter : position - opponentPenaltyAreaCenter;
    float rejection;
    // Apply maximal rejection if position is inside the penalty area
    if(std::abs(position.y()) < theFieldDimensions.yPosLeftPenaltyArea
       && ((ownPenaltyArea && position.x() < theFieldDimensions.xPosOwnPenaltyArea)
           || (!ownPenaltyArea && position.x() > theFieldDimensions.xPosOpponentPenaltyArea)))
    {
      rejection = maxRejection;
    }
    else
    {
      // Apply proportional rejection with distance to the penalty area
      const Vector2f& leftPenaltyAreaT = ownPenaltyArea ? ownLeftPenaltyAreaT : opponentLeftPenaltyAreaT;
      const Vector2f& rightPenaltyAreaT = ownPenaltyArea ? ownRightPenaltyAreaT : opponentRightPenaltyAreaT;
      const Vector2f& leftPenaltyAreaL = ownPenaltyArea ? ownLeftPenaltyAreaL : opponentLeftPenaltyAreaL;
      const Vector2f& rightPenaltyAreaL = ownPenaltyArea ? ownRightPenaltyAreaL : opponentRightPenaltyAreaL;
      const float distanceToFrontBorder = Geometry::getDistanceToEdge(Geometry::Line(leftPenaltyAreaL, rightPenaltyAreaL - leftPenaltyAreaL), position);
      const float distanceToLeftBorder = Geometry::getDistanceToEdge(Geometry::Line(leftPenaltyAreaL, (leftPenaltyAreaT - leftPenaltyAreaL)), position);
      const float distanceToRightBorder = Geometry::getDistanceToEdge(Geometry::Line(rightPenaltyAreaL, (rightPenaltyAreaT - rightPenaltyAreaL)), position);
      const float minDistance = std::min(std::min(distanceToLeftBorder, distanceToRightBorder), distanceToFrontBorder);
      rejection = mapToRange(minDistance, 0.0f, maxDistance, maxRejection, 0.0f);
    }
    return direction.normalized(rejection);
  };

  const auto computeVectorPotentialFieldBorderStrip = [&](const Vector2f& position) -> Vector2f
  {
    if(reducedRect.isInside(position))
      return Vector2f::Zero();
    Vector2f clippedPosition = position;
    reducedRect.clip(clippedPosition);
    const float rejection = std::min((clippedPosition - position).norm() / maxDistance, 1.f) * maxRejection;
    return (clippedPosition - position).normalized(rejection);
  };

  const auto computeVectorPotentialFieldOpponentHalf = [&](const Vector2f& position) -> Vector2f
  {
    float rejection;
    // Apply maximal rejection if position is inside the opponent half
    if(position.x() > 0.0f)
      rejection = maxRejection;
    else
    {
      // Apply proportional rejection with distance to the penalty area
      rejection = mapToRange(-position.x(), 0.0f, maxDistance, maxRejection, 0.0f);
    }
    return Vector2f(-rejection, 0.0f);
  };

  const auto computeVectorPotentialFieldCircle = [&](const Vector2f& position, const Vector2f& center, const float radius, const Vector2f& targetPosition = Vector2f(10000.0f, 0.0f)) -> Vector2f
  {
    // Apply proportional rejection with distance to the circle
    const float distance = std::max((position - center).norm() - radius, 0.0f);
    const float rejection = mapToRange(distance, 0.0f, maxDistance, maxRejection, 0.0f);
    if(targetPosition == Vector2f(10000.0f, 0.0f))
      return (position - center).normalized(rejection);
    // calculates the sum of two normalized vectors (own goal vector + shortest way vector) to leave the circle towards the own goal
    return ((position - center).normalized() + (targetPosition - center).normalized()).normalized(rejection);
  };

  const auto computeVectorPotentialFieldCenterCircle = [&](const Vector2f& position) -> Vector2f
  {
    return computeVectorPotentialFieldCircle(position, Vector2f::Zero(), theFieldDimensions.centerCircleRadius);
  };

  const auto computeVectorPotentialFieldBallArea = [&](const Vector2f& position) -> Vector2f
  {
    return computeVectorPotentialFieldCircle(position, theFieldBall.recentBallPositionOnField(), freeKickClearAreaRadius, Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.0f));
  };

  const auto computeVectorPotentialFieldNotOwnGoalLine = [&](const Vector2f& position) -> Vector2f
  {
    const Vector2f targetPoint(theFieldDimensions.xPosOwnGoalLine, 0);
    const float distance = (targetPoint - position).norm();
    const float rejection = mapToRange(distance, 0.f, maxDistance, 0.f, maxRejection);
    return (targetPoint - position).normalized(rejection);
  };

  const auto computeVectorPotentialFieldPlayer = [&](const Vector2f& position, const Vector2f& player) -> Vector2f
  {
    // rejection of the supported player
    const float rejection = 350.0f;
    // radius of rejection around the supported player in mm
    const float maxDistance = 800.0f;
    // directional vector between supported player and own robot
    const Vector2f difference = position - player;
    // rejection is weighted with the distance to the supported player, but limited to maxDistance
    const float length = std::max(0.0f, rejection - difference.norm() * rejection / maxDistance);
    return difference.normalized(length);
  };

  const auto calculatePotentialField = [&](const Vector2f& position, const Vector2f& target) -> Vector2f
  {
    const int duration = (theGameState.timeWhenStateEnds - theGameState.timeWhenStateStarted) / 2;
    unsigned illegal = theIllegalAreas.illegal;

    if(theIllegalAreas.willPositionBeIllegalIn(position, 150.0f, duration))
      illegal = illegal | theIllegalAreas.anticipatedIllegal;

    Vector2f result = computeVectorPotentialFieldTarget(position, target);
    if(illegal & bit(IllegalAreas::ownPenaltyArea))
      result += computeVectorPotentialFieldPenaltyArea(position, true);
    if(illegal & bit(IllegalAreas::opponentPenaltyArea))
      result += computeVectorPotentialFieldPenaltyArea(position, false);
    if(illegal & bit(IllegalAreas::borderStrip))
      result += computeVectorPotentialFieldBorderStrip(position);
    if(illegal & bit(IllegalAreas::opponentHalf))
      result += computeVectorPotentialFieldOpponentHalf(position);
    if(illegal & bit(IllegalAreas::centerCircle))
    {
      Vector2f p = computeVectorPotentialFieldCenterCircle(position);
      // If we have to leave the opponent half, we don't wont the center circle field to push us into it (local minimum). ~ German Open 2024
      if((illegal & bit(IllegalAreas::opponentHalf)) && p.x() > 0.f)
        p.x() = 0.f;
      result += p;
    }
    if(illegal & bit(IllegalAreas::ballArea))
      result += computeVectorPotentialFieldBallArea(position);
    if(illegal & bit(IllegalAreas::notOwnGoalLine))
      result += computeVectorPotentialFieldNotOwnGoalLine(position);
    for(const auto& teammate : theGlobalTeammatesModel.teammates)
      result += computeVectorPotentialFieldPlayer(position, teammate.pose.translation);
    return result;
  };

  const auto getPotentialField = [&](const Vector2f& target) -> Vector2f
  {
    return calculatePotentialField(theRobotPose.translation, target).rotate(-theRobotPose.rotation);
  };

  const auto draw = [&]
  {
    Vector2f target = Vector2f::Zero();
    MODIFY("option:WalkPotentialField:target", target);

    COMPLEX_DRAWING("option:WalkPotentialField:angles")
    {
      const Vector2f rp = theRobotPose.translation;
      const Vector2f t = theRobotPose.inverse() * target;

      const Vector2f end(rp + calculatePotentialField(rp, target).normalize(t.norm()));
      ARROW("option:WalkPotentialField:angles", rp.x(), rp.y(), end.x(), end.y(), 6, Drawings::solidPen, ColorRGBA::yellow);
      ARROW("option:WalkPotentialField:angles", rp.x(), rp.y(), target.x(), target.y(), 6, Drawings::solidPen, ColorRGBA::green);
      const Vector2f pt = getPotentialField(target);
      const float a = toDegrees(std::acos(std::abs(clip<float>((pt.dot(t)) / (pt.norm() * t.norm()), -1.0f, 1.0f))));
      DRAW_TEXT("option:WalkPotentialField:angles", rp.x(), rp.y(), 200, ColorRGBA::white, a);
    }

    COMPLEX_DRAWING("option:WalkPotentialField:potentialField")
    {
      for(float y = theFieldDimensions.yPosRightFieldBorder; y <= theFieldDimensions.yPosLeftFieldBorder; y += gridOffset)
      {
        for(float x = theFieldDimensions.xPosOwnFieldBorder; x <= theFieldDimensions.xPosOpponentFieldBorder; x += gridOffset)
        {
          const Vector2f start(x, y);
          const Vector2f end(start + calculatePotentialField(start, target));
          ARROW("option:WalkPotentialField:potentialField", start.x(), start.y(), end.x(), end.y(), arrowWidth, Drawings::solidPen, ColorRGBA::black);
        }
      }
    }
  };

  const Vector2f potentialTargetRel = getPotentialField(target);

  common_transition
  {
    if(straight)
      goto straight;
    else
      goto omnidirectional;
  }

  initial_state(straight)
  {
    action
    {
      WalkToPoint({.target = {0.f, potentialTargetRel},
                   .reduceWalkingSpeed = ReduceWalkSpeedType::noChange,
                   .targetOfInterest = targetOfInterest});
      if(targetOfInterest)
        LookAtBallAndTarget({.startBall = true,
                             .walkingDirection = potentialTargetRel});
      draw();
    }
  }

  state(omnidirectional)
  {
    action
    {
      const Vector2f ballPositionRel = theFieldBall.recentBallPositionRelative(3000);
      float walkAngle = Angle::normalize(potentialTargetRel.angle() + ballFactor * (ballPositionRel.angle() - potentialTargetRel.angle()));
      if(std::abs(walkAngle - ballPositionRel.angle()) > maxBallAngleOffset)
        walkAngle = Angle::normalize(walkAngle > 0.f ? ballPositionRel.angle() + maxBallAngleOffset : ballPositionRel.angle() - maxBallAngleOffset);
      if(potentialTargetRel.squaredNorm() < sqr(alignDistance))
      {
        if(useRotation)
          walkAngle = rotation;
        else
          walkAngle = ballPositionRel.angle();
      }

      WalkToPoint({.target = {walkAngle, potentialTargetRel},
                   .reduceWalkingSpeed = ReduceWalkSpeedType::noChange,
                   .disableAligning = std::abs(walkAngle) > 10_deg || useRotation,
                   .targetOfInterest = targetOfInterest});

      if(targetOfInterest)
        LookAtBallAndTarget({.startBall = true,
                             .walkingDirection = potentialTargetRel});
    }
  }

  draw();
}
