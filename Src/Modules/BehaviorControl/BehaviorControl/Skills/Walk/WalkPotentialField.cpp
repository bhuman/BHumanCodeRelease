/**
 * @file WalkPotentialField.cpp
 *
 * This file implements an implementation for the WalkPotentialField skill.
 *
 * @author Andreas Stolpmann
 * @author somebody unknown
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Libraries/LibTeam.h"
#include "Representations/BehaviorControl/Libraries/LibTeammates.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Settings.h"
#include <cmath>

SKILL_IMPLEMENTATION(WalkPotentialFieldImpl,
{,
  IMPLEMENTS(WalkPotentialField),
  CALLS(WalkToPoint),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(LibTeam),
  REQUIRES(LibTeammates),
  REQUIRES(RobotPose),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(TeamData),
  DEFINES_PARAMETERS(
  {,
    (float)(250.f) alignDistance, /**< If the target is closer than this distance, the robot is completely aligned to the ball or the target rotation. */
    (Angle)(45_deg) maxBallAngleOffset, /**< If not walking straight, the ball should have at most this angle relative to the robot's orientation. */
  }),
});

class WalkPotentialFieldImpl : public WalkPotentialFieldImplBase
{
  void execute(const WalkPotentialField& p) override
  {
    const Vector2f potentialTargetRel = getPotentialField(p.target, p.playerNumber);
    if(p.straight)
      theWalkToPointSkill(Pose2f(potentialTargetRel.angle(), potentialTargetRel));
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

      theWalkToPointSkill(Pose2f(walkAngle, potentialTargetRel), 1.f, /* rough: */ false, /* disableObstacleAvoidance: */ false, /* disableAligning: */ std::abs(walkAngle) > 10_deg || p.useRotation);
    }
  }

  void preProcess(const WalkPotentialField&) override
  {
    DECLARE_DEBUG_DRAWING("skill:WalkPotentialFieldImpl:angles", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:WalkPotentialFieldImpl:potentialField", "drawingOnField");
    draw();
  }

  void preProcess() override {}

  Vector2f getPotentialField(const Vector2f& target, int player)
  {
    return calculatePotentialField(theRobotPose.translation, target, player).rotate(-theRobotPose.rotation);
  }

  Vector2f calculatePotentialField(const Vector2f& pos, const Vector2f& target, int toSupport)
  {
    Vector2f result = computeVectorPotentialFieldTarget(pos, target);

    if(toSupport >= Settings::lowestValidPlayerNumber && toSupport <= Settings::highestValidPlayerNumber)
      result += computeVectorPotentialFieldToSupportBallLine(pos, toSupport) +
                computeVectorPotentialFieldBallGoalLine(pos, toSupport) +
                computeVectorPotentialFieldBehindToSupport(pos, toSupport);

    if(!theTeamBehaviorStatus.role.isGoalkeeper() && theLibTeammates.nonKeeperTeammatesInOwnPenaltyArea >= 2)
      result += computeVectorPotentialFieldPenaltyArea(true, pos);
    if(theLibTeammates.teammatesInOpponentPenaltyArea >= 3)
      result += computeVectorPotentialFieldPenaltyArea(false, pos);
    for(auto const& teammate : theTeamData.teammates)
    {
      if(teammate.status != Teammate::PENALIZED && !teammate.theTeamBehaviorStatus.role.isGoalkeeper())
        result += computeVectorPotentialFieldPlayer(pos, teammate.theRobotPose.translation);
    }
    return result;
  }

  Vector2f computeVectorPotentialFieldToSupportBallLine(const Vector2f& pos, int toSupport) const
  {
    const float toSupportBallDistance = theLibTeam.getBallPosition(toSupport).norm();
    const float maxBallDistance = 1200.0f;
    // toSupport position on the field
    const Vector2f toSupportPosition = theLibTeam.getTeammatePose(toSupport).translation;
    // ball position on the field
    const Vector2f ballPosition = theFieldBall.recentBallPositionOnField();
    // construct line from ball to goal
    Geometry::Line line(toSupportPosition, ballPosition - toSupportPosition);
    // distance to that line
    const float distance = Geometry::getDistanceToEdge(line, pos);
    // compute whether we are on the left (positive value) or the right side (negative value) of the line
    const float side = (ballPosition.x() - toSupportPosition.x()) * (pos.y() - toSupportPosition.y()) - (pos.x() - toSupportPosition.x()) * (ballPosition.y() - toSupportPosition.y());
    // rejection is always orthogonal to that line
    if(side > 0)
    {
      line.base = Pose2f(line.direction.angle(), line.base).translate(0.f, 100.f).translation;
      line.direction.rotateLeft();
    }
    else
    {
      line.base = Pose2f(line.direction.angle(), line.base).translate(0.f, -100.f).translation;
      line.direction.rotateRight();
    }
    // compute rejection based on distance to the line and ball distance of supported player
    const float maxDistance = 750.0f;
    const float rejection = std::max(0.0f, (maxBallDistance - toSupportBallDistance) * 400.0f / maxBallDistance);
    const float length = std::max(0.0f, rejection - distance * rejection / maxDistance);
    return line.direction.normalize(length);
  }

  Vector2f computeVectorPotentialFieldBallGoalLine(const Vector2f& pos, int toSupport) const
  {
    const float toSupportBallDistance = theLibTeam.getBallPosition(toSupport).norm();
    const float maxBallDistance = 1200.0f;
    // ball position on the field
    Vector2f ballPosition = theFieldBall.recentBallPositionOnField();
    Vector2f oppGoalPosition(goalPosition);
    // construct line from ball to goal
    Geometry::Line line(ballPosition, oppGoalPosition - ballPosition);
    // distance to that line
    const float distance = Geometry::getDistanceToEdge(line, pos);
    // compute whether we are on the left (positive value) or the right side (negative value) of the line
    const float side = (oppGoalPosition.x() - ballPosition.x()) * (pos.y() - ballPosition.y()) - (pos.x() - ballPosition.x()) * (oppGoalPosition.y() - ballPosition.y());
    // rejection is always orthogonal to that line
    if(side > 0)
    {
      ballPosition.y() += 100.f;
      oppGoalPosition.y() = theFieldDimensions.yPosLeftGoal;
      line = Geometry::Line(ballPosition, oppGoalPosition - ballPosition);
      line.direction.rotateLeft();
    }
    else
    {
      ballPosition.y() -= 100.f;
      oppGoalPosition.y() = theFieldDimensions.yPosRightGoal;
      line = Geometry::Line(ballPosition, oppGoalPosition - ballPosition);
      line.direction.rotateRight();
    }
    // compute rejection based on distance to the line and ball distance of supported player
    const float maxDistance = 1000.0f;
    const float rejection = std::max(0.0f, (maxBallDistance - toSupportBallDistance) * 400.0f / maxBallDistance);
    const float length = std::max(0.0f, rejection - distance * rejection / maxDistance);
    return line.direction.normalize(length);
  }

  Vector2f computeVectorPotentialFieldPlayer(const Vector2f& pos, const Vector2f& playerPos) const
  {
    // rejection of the supported player
    const float rejection = 350.0f;
    // radius of rejection around the supported player in mm
    const float maxDistance = 800.0f;
    // directional vector between supported player and own robot
    Vector2f difference = pos - playerPos;
    // rejection is weighted with the distance to the supported player, but limited to maxDistance
    const float length = std::max(0.0f, rejection - difference.norm() * rejection / maxDistance);
    return difference.normalize(length);
  }

  Vector2f computeVectorPotentialFieldBehindToSupport(const Vector2f& pos, int toSupport) const
  {
    const Vector2f supportedPlayerPos = theLibTeam.getTeammatePose(toSupport).translation;
    //if the supporter is directly in front of the to be supported player this potential field should be zero
    if(supportedPlayerPos.x() < pos.x() && std::abs(supportedPlayerPos.y() - pos.y()) < 100)
    {
      return Vector2f::Zero();
    }
    // rejection of the supported player
    const float rejection = 350.0f;
    // radius of rejection around the supported player in mm
    const float maxDistance = 800.0f;
    //vector pointing backwards
    Vector2f backwards = Vector2f(-1 * std::abs(pos.x() - theLibTeam.getTeammatePose(toSupport).translation.x()), 0);
    float distToSupportedPlayer = (supportedPlayerPos - pos).norm();
    // rejection is weighted with the distance to the supported player, but limited to maxDistance
    const float length = std::max(0.0f, rejection - distToSupportedPlayer * rejection / maxDistance);
    return backwards.normalize(length);
  }

  Vector2f computeVectorPotentialFieldTarget(const Vector2f& pos, const Vector2f& target) const
  {
    // radius around the target where slowing down is started
    const float slowDownRadius = 500.0f;
    const float maxSpeed = 200.0f;
    // directional vector to target
    Vector2f difference = target - pos;
    // attraction is weighted with distance to target, but clipped with slow down radius
    const float length = std::min(maxSpeed, difference.norm() * maxSpeed / slowDownRadius);
    return difference.normalize(length);
  }

  Vector2f computeVectorPotentialFieldPenaltyArea(const bool ownPenaltyArea, const Vector2f& pos) const
  {
    const float maxRejection = 600;
    const float maxDistance = 400;
    Vector2f diff(pos - centerOfRejection);
    if(!ownPenaltyArea)
      diff = pos + centerOfRejection;

    float rejection;
    // if inside penalty area maximal rejection
    if(((ownPenaltyArea && pos.x() < theFieldDimensions.xPosOwnPenaltyArea)
        || (!ownPenaltyArea && pos.x() > theFieldDimensions.xPosOpponentPenaltyArea))
        && std::abs(pos.y()) < theFieldDimensions.yPosLeftPenaltyArea)
    {
      rejection = maxRejection;
    }
    else
    {
      // rejection proportional to distance to penalty area
      if(ownPenaltyArea)
      {
        const float distanceToFrontBorder = Geometry::getDistanceToEdge(Geometry::Line(ownLeftCorner, ownRightCorner - ownLeftCorner), pos);
        const float distanceToLeftBorder = Geometry::getDistanceToEdge(Geometry::Line(ownLeftCorner, -2000.0f), pos);
        const float distanceToRightBorder = Geometry::getDistanceToEdge(Geometry::Line(ownRightCorner, -2000.0f), pos);
        const float minDistance = std::min(std::min(distanceToLeftBorder, distanceToRightBorder), distanceToFrontBorder);
        rejection = std::max(maxDistance - minDistance, 0.0f) / maxDistance * maxRejection;
      }
      else
      {
        const float distanceToFrontBorder = Geometry::getDistanceToEdge(Geometry::Line(opponentLeftCorner, opponentRightCorner - opponentLeftCorner), pos);
        const float distanceToLeftBorder = Geometry::getDistanceToEdge(Geometry::Line(opponentLeftCorner, -2000.0f), pos);
        const float distanceToRightBorder = Geometry::getDistanceToEdge(Geometry::Line(opponentRightCorner, -2000.0f), pos);
        const float minDistance = std::min(std::min(distanceToLeftBorder, distanceToRightBorder), distanceToFrontBorder);
        rejection = std::max(maxDistance - minDistance, 0.0f) / maxDistance * maxRejection;
      }
    }
    return diff.normalize(rejection);
  }

  void draw()
  {
    int toSupport = -1;
    Vector2f target = Vector2f::Zero();
    MODIFY("skill:WalkPotentialFieldImpl:toSupport", toSupport);
    MODIFY("skill:WalkPotentialFieldImpl:target", target);

    COMPLEX_DRAWING("skill:WalkPotentialFieldImpl:angles")
    {
      const Vector2f rp = theRobotPose.translation;
      const Vector2f t = theRobotPose.inversePose * target;

      const Vector2f end(rp + calculatePotentialField(rp, target, toSupport).normalize(t.norm()));
      ARROW("skill:WalkPotentialFieldImpl:angles", rp.x(), rp.y(), end.x(), end.y(), 6, Drawings::solidPen, ColorRGBA::yellow);
      ARROW("skill:WalkPotentialFieldImpl:angles", rp.x(), rp.y(), target.x(), target.y(), 6, Drawings::solidPen, ColorRGBA::green);
      const Vector2f pt = getPotentialField(target, toSupport);
      const float a = toDegrees(std::acos(std::abs(clip<float>((pt.dot(t)) / (pt.norm() * t.norm()), -1.0f, 1.0f))));
      DRAW_TEXT("skill:WalkPotentialFieldImpl:angles", rp.x(), rp.y(), 200, ColorRGBA::white, a);
    }

    COMPLEX_DRAWING("skill:WalkPotentialFieldImpl:potentialField")
    {
      const float off = 100.0f;
      if(toSupport != -1)
      {
        for(float y = theFieldDimensions.yPosRightFieldBorder; y <= theFieldDimensions.yPosLeftFieldBorder; y += off)
        {
          for(float x = theFieldDimensions.xPosOwnFieldBorder; x <= theFieldDimensions.xPosOpponentFieldBorder; x += off)
          {
            const Vector2f start(x, y);
            const Vector2f end(start + calculatePotentialField(start, target, toSupport));
            ARROW("skill:WalkPotentialFieldImpl:potentialField", start.x(), start.y(), end.x(), end.y(), 2, Drawings::solidPen, ColorRGBA::black);
          }
        }
      }
    }
  }

  const Vector2f goalPosition = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f);
  const Vector2f centerOfRejection = Vector2f(theFieldDimensions.xPosOwnGroundLine - 1000.0f, 0.0f);
  const Vector2f ownLeftCorner = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f ownRightCorner = Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
  const Vector2f opponentLeftCorner = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea);
  const Vector2f opponentRightCorner = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea);
};

MAKE_SKILL_IMPLEMENTATION(WalkPotentialFieldImpl);
