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

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/IllegalAreas.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Debugging.h"
#include "Math/Angle.h"
#include "Math/Geometry.h"
#include "Math/BHMath.h"
#include "Framework/Settings.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include <cmath>

SKILL_IMPLEMENTATION(WalkPotentialFieldImpl,
{,
  IMPLEMENTS(WalkPotentialField),
  CALLS(WalkToPoint),
  CALLS(LookAtBallAndTarget),
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(IllegalAreas),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(GameState),
  DEFINES_PARAMETERS(
  {,
    (float)(250.f) alignDistance, /**< If the target is closer than this distance, the robot is completely aligned to the ball or the target rotation. */
    (Angle)(45_deg) maxBallAngleOffset, /**< If not walking straight, the ball should have at most this angle relative to the robot's orientation. */
    (float)(600.f) maxRejection,
    (float)(400.f) maxDistance,
    (float)(750.f) freeKickClearAreaRadius, /**< The radius of the area that has to be cleared by the defending team during a free kick. */
    (float)(350.f) gridOffset, /**< Spacing for the origins of the arrows in the debug drawing of the potential field */
    (int)(15) arrowWidth, /**< Pen width for the arrow lines in the debug drawing of the potential field */
  }),
});

class WalkPotentialFieldImpl : public WalkPotentialFieldImplBase
{
  void execute(const WalkPotentialField& p) override
  {
    const Vector2f potentialTargetRel = getPotentialField(p.target);
    if(p.straight)
    {
      theWalkToPointSkill({.target = {0.f, potentialTargetRel},
                           .reduceWalkingSpeed = false,
                           .targetOfInterest = p.targetOfInterest});
      if(p.targetOfInterest)
        theLookAtBallAndTargetSkill({.startTarget = true,
                                     .walkingDirection = potentialTargetRel,
                                     .ballPositionAngle = theFieldBall.recentBallPositionRelative().angle() });
    }
    else
    {
      const Vector2f ballPositionRel = theFieldBall.recentBallPositionRelative(3000);
      float walkAngle = Angle::normalize(potentialTargetRel.angle() + p.ballFactor * (ballPositionRel.angle() - potentialTargetRel.angle()));
      if(std::abs(walkAngle - ballPositionRel.angle()) > maxBallAngleOffset)
        walkAngle = Angle::normalize(walkAngle > 0.f ? ballPositionRel.angle() + maxBallAngleOffset : ballPositionRel.angle() - maxBallAngleOffset);
      if(potentialTargetRel.squaredNorm() < sqr(alignDistance))
      {
        if(p.useRotation)
          walkAngle = p.rotation;
        else
          walkAngle = ballPositionRel.angle();
      }

      theWalkToPointSkill({.target = {walkAngle, potentialTargetRel},
                           .reduceWalkingSpeed = false,
                           .disableAligning = std::abs(walkAngle) > 10_deg || p.useRotation,
                           .targetOfInterest = p.targetOfInterest });

      if(p.targetOfInterest)
        theLookAtBallAndTargetSkill({.startTarget = true,
                                     .walkingDirection = potentialTargetRel,
                                     .ballPositionAngle = theFieldBall.recentBallPositionRelative().angle() });
    }
  }

  void preProcess(const WalkPotentialField&) override
  {
    DECLARE_DEBUG_DRAWING("skill:WalkPotentialFieldImpl:angles", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:WalkPotentialFieldImpl:potentialField", "drawingOnField");
    // TODO: This should be called every frame, right?
    draw();
  }

  void preProcess() override {}

  Vector2f getPotentialField(const Vector2f& target)
  {
    return calculatePotentialField(theRobotPose.translation, target).rotate(-theRobotPose.rotation);
  }

  Vector2f calculatePotentialField(const Vector2f& position, const Vector2f& target)
  {
    const float duration = (theGameState.timeWhenStateEnds - theGameState.timeWhenStateStarted) * 0.5f;
    unsigned illegal = theIllegalAreas.illegal;

    if(theIllegalAreas.willPositionBeIllegalIn(position, 150.0f, duration))
      illegal = illegal | theIllegalAreas.anticipatedIllegal;

    Vector2f result = computeVectorPotentialFieldTarget(position, target);
    if((illegal & bit(IllegalAreas::ownPenaltyArea)))
      result += computeVectorPotentialFieldPenaltyArea(position, true);
    if((illegal & bit(IllegalAreas::opponentPenaltyArea)))
      result += computeVectorPotentialFieldPenaltyArea(position, false);
    if((illegal & bit(IllegalAreas::borderStrip)))
      result += computeVectorPotentialFieldBorderStrip(position);
    if((illegal & bit(IllegalAreas::opponentHalf)))
      result += computeVectorPotentialFieldOpponentHalf(position);
    if((illegal & bit(IllegalAreas::centerCircle)))
      result += computeVectorPotentialFieldCenterCircle(position);
    if((illegal & bit(IllegalAreas::ballArea)))
      result += computeVectorPotentialFieldBallArea(position);
    if((illegal & bit(IllegalAreas::notOwnGoalLine)))
      result += computeVectorPotentialFieldNotOwnGoalLine(position);
    for(auto const& teammate : theTeamData.teammates)
    {
      if(teammate.theStrategyStatus.role != PositionRole::toRole(PositionRole::goalkeeper))
        result += computeVectorPotentialFieldPlayer(position, teammate.getEstimatedPosition(theFrameInfo.time));
    }
    return result;
  }

  Vector2f computeVectorPotentialFieldPlayer(const Vector2f& position, const Vector2f& player) const
  {
    // rejection of the supported player
    const float rejection = 350.0f;
    // radius of rejection around the supported player in mm
    const float maxDistance = 800.0f;
    // directional vector between supported player and own robot
    Vector2f difference = position - player;
    // rejection is weighted with the distance to the supported player, but limited to maxDistance
    const float length = std::max(0.0f, rejection - difference.norm() * rejection / maxDistance);
    return difference.normalize(length);
  }


  Vector2f computeVectorPotentialFieldTarget(const Vector2f& position, const Vector2f& target) const
  {
    // radius around the target where slowing down is started
    const float slowDownRadius = 500.0f;
    const float maxSpeed = 200.0f;
    // directional vector to target
    Vector2f difference = target - position;
    // attraction is weighted with distance to target, but clipped with slow down radius
    const float length = std::min(maxSpeed, difference.norm() * maxSpeed / slowDownRadius);
    return difference.normalize(length);
  }

  Vector2f computeVectorPotentialFieldPenaltyArea(const Vector2f& position, const bool ownPenaltyArea) const
  {
    Vector2f direction = ownPenaltyArea
                         ? position - ownPenaltyAreaCenter
                         : position - opponentPenaltyAreaCenter;
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
      const Vector2f& leftCorner = ownPenaltyArea ? ownLeftCorner : opponentLeftCorner;
      const Vector2f& rightCorner = ownPenaltyArea ? ownRightCorner : opponentRightCorner;
      const Vector2f& leftPenaltyArea = ownPenaltyArea ? ownLeftPenaltyArea : opponentLeftPenaltyArea;
      const Vector2f& rightPenaltyArea = ownPenaltyArea ? ownRightPenaltyArea : opponentRightPenaltyArea;
      const float distanceToFrontBorder = Geometry::getDistanceToEdge(Geometry::Line(leftCorner, rightCorner - leftCorner), position);
      const float distanceToLeftBorder = Geometry::getDistanceToEdge(Geometry::Line(leftCorner, (leftPenaltyArea - leftCorner)), position);
      const float distanceToRightBorder = Geometry::getDistanceToEdge(Geometry::Line(rightCorner, (rightPenaltyArea - rightCorner)), position);
      const float minDistance = std::min(std::min(distanceToLeftBorder, distanceToRightBorder), distanceToFrontBorder);
      rejection = mapToRange(minDistance, 0.0f, maxDistance, maxRejection, 0.0f);
    }
    return direction.normalize(rejection);
  }

  Vector2f computeVectorPotentialFieldOpponentHalf(const Vector2f& position) const
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
  }

  Vector2f computeVectorPotentialFieldCenterCircle(const Vector2f& position) const
  {
    return computeVectorPotentialFieldCircle(position, Vector2f::Zero(), theFieldDimensions.centerCircleRadius);
  }

  Vector2f computeVectorPotentialFieldBallArea(const Vector2f& position) const
  {
    return computeVectorPotentialFieldCircle(position, theFieldBall.recentBallPositionOnField(), freeKickClearAreaRadius, Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.0f));
  }

  Vector2f computeVectorPotentialFieldCircle(const Vector2f& position, const Vector2f& center, const float radius, const Vector2f& targetPosition = Vector2f(10000.0f, 0.0f)) const
  {
    // Apply proportional rejection with distance to the circle
    const float distance = std::max((position - center).norm() - radius, 0.0f);
    const float rejection = mapToRange(distance, 0.0f, maxDistance, maxRejection, 0.0f);
    if (targetPosition == Vector2f(10000.0f, 0.0f)) {
      return (position - center).normalized(rejection);
    }
    // calculates the sum of two normalized vectors (own goal vector + shortest way vector) to leave the circle towards the own goal
    return ((position - center).normalized() + (targetPosition - center).normalized()).normalized(rejection);
  }

  Vector2f computeVectorPotentialFieldBorderStrip(const Vector2f& position) const
  {
    const bool opponentFieldBorderStrip = position.x() > theFieldDimensions.xPosOpponentGroundLine;
    const bool ownFieldBorderStrip = position.x() < theFieldDimensions.xPosOwnGroundLine;
    const bool leftFieldBorderStrip = position.y() > theFieldDimensions.yPosLeftSideline;
    const bool rightFieldBorderStrip = position.y() < theFieldDimensions.yPosRightSideline;
    float rejection;
    // Apply maximal rejection if position is inside the field border strip
    if(opponentFieldBorderStrip || ownFieldBorderStrip || leftFieldBorderStrip || rightFieldBorderStrip)
      rejection = maxRejection;
    else
    {
      // Apply proportional rejection with distance to the border strip
      const float distance = std::min(std::max(0.f, theFieldDimensions.xPosOpponentGroundLine - std::abs(position.x())),
                                      std::max(0.f, theFieldDimensions.yPosLeftSideline - std::abs(position.y())));
      rejection = mapToRange(distance, 0.0f, maxDistance, maxRejection, 0.0f);
    }
    Vector2f direction = Vector2f::Zero();
    if(opponentFieldBorderStrip)
      direction += Vector2f(-rejection, 0.0f);
    if(ownFieldBorderStrip)
      direction += Vector2f(rejection, 0.0f);
    if(leftFieldBorderStrip)
      direction += Vector2f(0.0f, -rejection);
    if(rightFieldBorderStrip)
      direction += Vector2f(0.0f, rejection);
    return direction;
  }

  Vector2f computeVectorPotentialFieldNotOwnGoalLine(const Vector2f& position) const
  {
    // Apply proportional rejection with distance to the own goal line
    Geometry::Line line(ownLeftGoalPost, ownRightGoalPost - ownLeftGoalPost);
    const float distance = Geometry::getDistanceToEdge(line, position);
    const float rejection = mapToRange(distance, theFieldDimensions.fieldLinesWidth * 0.5f + 200.f, maxDistance, 0.f, maxRejection);
    const Vector2f point = Geometry::getOrthogonalProjectionOfPointOnEdge(line, position);
    return (point - position).normalized(rejection);
  }

  void draw()
  {
    Vector2f target = Vector2f::Zero();
    MODIFY("skill:WalkPotentialFieldImpl:target", target);

    COMPLEX_DRAWING("skill:WalkPotentialFieldImpl:angles")
    {
      const Vector2f rp = theRobotPose.translation;
      const Vector2f t = theRobotPose.inverse() * target;

      const Vector2f end(rp + calculatePotentialField(rp, target).normalize(t.norm()));
      ARROW("skill:WalkPotentialFieldImpl:angles", rp.x(), rp.y(), end.x(), end.y(), 6, Drawings::solidPen, ColorRGBA::yellow);
      ARROW("skill:WalkPotentialFieldImpl:angles", rp.x(), rp.y(), target.x(), target.y(), 6, Drawings::solidPen, ColorRGBA::green);
      const Vector2f pt = getPotentialField(target);
      const float a = toDegrees(std::acos(std::abs(clip<float>((pt.dot(t)) / (pt.norm() * t.norm()), -1.0f, 1.0f))));
      DRAW_TEXT("skill:WalkPotentialFieldImpl:angles", rp.x(), rp.y(), 200, ColorRGBA::white, a);
    }

    COMPLEX_DRAWING("skill:WalkPotentialFieldImpl:potentialField")
    {
      for(float y = theFieldDimensions.yPosRightFieldBorder; y <= theFieldDimensions.yPosLeftFieldBorder; y += gridOffset)
      {
        for(float x = theFieldDimensions.xPosOwnFieldBorder; x <= theFieldDimensions.xPosOpponentFieldBorder; x += gridOffset)
        {
          const Vector2f start(x, y);
          const Vector2f end(start + calculatePotentialField(start, target));
          ARROW("skill:WalkPotentialFieldImpl:potentialField", start.x(), start.y(), end.x(), end.y(), arrowWidth, Drawings::solidPen, ColorRGBA::black);
        }
      }
    }
  }

  const Vector2f opponentGoalPosition = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f);
  const Vector2f ownGoalPosition = Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f);
  const Vector2f ownLeftGoalPost = Vector2f(theFieldDimensions.xPosOwnGoalPost + theFieldDimensions.fieldLinesWidth * 0.5f, theFieldDimensions.yPosLeftGoal - theBallSpecification.radius * 2.f);
  const Vector2f ownRightGoalPost = Vector2f(theFieldDimensions.xPosOwnGoalPost + theFieldDimensions.fieldLinesWidth * 0.5f, theFieldDimensions.yPosRightGoal + theBallSpecification.radius * 2.f);
  const Vector2f ownPenaltyAreaCenter = Vector2f(theFieldDimensions.xPosOwnGroundLine - 1000.0f, 0.0f);
  const Vector2f opponentPenaltyAreaCenter = Vector2f(theFieldDimensions.xPosOpponentGroundLine + 1000.0f, 0.0f);
  const Vector2f ownLeftCorner = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f ownRightCorner = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f opponentLeftCorner = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f opponentRightCorner = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f ownLeftPenaltyArea = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f ownRightPenaltyArea = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f opponentLeftPenaltyArea = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f opponentRightPenaltyArea = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
};

MAKE_SKILL_IMPLEMENTATION(WalkPotentialFieldImpl);
