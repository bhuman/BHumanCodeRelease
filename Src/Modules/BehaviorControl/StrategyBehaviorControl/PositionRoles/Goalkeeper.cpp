/**
 * @file Goalkeeper.cpp
 *
 * This file implements the goalkeeper role.
 *
 * @author Arne Hasselbring
 */

#include "Goalkeeper.h"
#include "Framework/Settings.h"
#include "Math/Geometry.h"
#include "Math/Pose2f.h"
#include "Platform/SystemCall.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Modify.h"

//Defines the parameters and debug drawings for the goalkeeper
void Goalkeeper::preProcess()
{
  MODIFY("behavior:Goalkeeper", p);
  DECLARE_DEBUG_DRAWING("behavior:Goalkeeper:pose", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Goalkeeper:coverage", "drawingOnField");
}

//calculates and sets the position of the goalkeeper. Its called every frame, when the goalkeeper is active
Pose2f Goalkeeper::position(Side, const Pose2f& basePose, const std::vector<Vector2f>&, const Agent&, const Agents&)
{
  //make sure, the goalkeeper has an offset to the goal line
  const float minXValue = theFieldDimensions.xPosOwnGoalLine + p.distanceToGoalLineForMinimumXValue;
  p.goaliePoseField = calcGlobalPoseOnBobLineAndCut();
  ASSERT(p.goaliePoseField.isFinite());
  if(p.goaliePoseField.translation.x() < minXValue)
    p.goaliePoseField.translation.x() = minXValue;

  p.goaliePoseRel = theRobotPose.inverse() * p.goaliePoseField;

  draw();
  verify();

  return theGameState.isGoalKick() && theGameState.isForOwnTeam() ? basePose : p.goaliePoseField;
}

//Calculates the debug drawings for the goalkeeper and the goaliepose
void Goalkeeper::draw() const
{
  CROSS("behavior:Goalkeeper:pose",
        p.goaliePoseField.translation.x(),
        p.goaliePoseField.translation.y(), 40, 20, Drawings::solidPen, ColorRGBA::red);

  DEBUG_DRAWING("behavior:Goalkeeper:coverage", "drawingOnField")
  {
    const Vector2f& gloBallPos = theFieldBall.positionOnField;

    const Vector2f offset = Vector2f(0.f, 1.f).rotate(p.goaliePoseField.rotation);

    const float standRange = 80.f;
    const float genuflectRange = 200.f;
    const float jumpRange = 600.f;

    auto drawOffset = [&](const float range, [[maybe_unused]] const ColorRGBA color)
    {
      const Vector2f rangeOffset = offset.normalized(range);
      const Vector2f keeperLeft = p.goaliePoseField.translation + rangeOffset;
      const Vector2f keeperRight = p.goaliePoseField.translation - rangeOffset;

      const Geometry::Line leftLine(gloBallPos, keeperLeft - gloBallPos);
      const Geometry::Line rightLine(gloBallPos, keeperRight - gloBallPos);

      const Geometry::Line goalLine(Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f), Vector2f(0.f, 1.f));

      Vector2f useKeeperLeft;
      Vector2f useKeeperRight;
      Vector2f intersections;

      //calculate isNearPost
      const auto [isNearLeftPost, isNearRightPost] = theLibPosition.isNearPost(p.goaliePoseField);

      if(!Geometry::getIntersectionOfLines(goalLine, leftLine, intersections) || std::abs(intersections.y()) > theFieldDimensions.yPosLeftGoal)
        useKeeperLeft = isNearLeftPost ? p.goaliePoseField.translation : Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftGoal);
      else
        useKeeperLeft = intersections;

      if(!Geometry::getIntersectionOfLines(goalLine, rightLine, intersections) || std::abs(intersections.y()) > theFieldDimensions.yPosLeftGoal)
        useKeeperRight = isNearRightPost ? p.goaliePoseField.translation : Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightGoal);
      else
        useKeeperRight = intersections;

      const Vector2f points[4] = { keeperLeft, keeperRight, useKeeperRight, useKeeperLeft };
      POLYGON("behavior:Goalkeeper:coverage", 4, points, 10, Drawings::noPen, color, Drawings::solidBrush, color);
    };

    const Vector2f points[3] = { gloBallPos, Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightGoal), Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftGoal) };
    POLYGON("behavior:Goalkeeper:coverage", 3, points, 10, Drawings::noPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::red);

    drawOffset(jumpRange, ColorRGBA::orange);
    drawOffset(genuflectRange, ColorRGBA::violet);
    drawOffset(standRange, ColorRGBA::green);
    LINE("behavior:Goalkeeper:coverage", p.goaliePoseField.translation.x(), p.goaliePoseField.translation.y(), gloBallPos.x(), gloBallPos.y(), 50.f, Drawings::dashedPen, ColorRGBA::black);
  }
}

//Verifies that the goalkeeper position is valid
void Goalkeeper::verify() const
{
  ASSERT(p.goaliePoseField.isFinite());
  ASSERT(p.goaliePoseField.translation.x() > -6500.f);
  ASSERT(p.goaliePoseField.translation.x() < -1000.f);
  ASSERT(p.goaliePoseField.translation.y() > -3000.f);
  ASSERT(p.goaliePoseField.translation.y() < 3000.f);
  ASSERT(p.goaliePoseField.rotation >= -pi);
  ASSERT(p.goaliePoseField.rotation <= pi);
  ASSERT(p.goaliePoseRel.isFinite());
  ASSERT(p.goaliePoseRel.rotation >= -pi);
  ASSERT(p.goaliePoseRel.rotation <= pi);
};

Pose2f Goalkeeper::tolerance() const
{
  const float distanceFactor = mapToRange(theFieldBall.recentBallPositionRelative().norm(), p.ballDistanceInterpolationRange.min, p.ballDistanceInterpolationRange.max, 0.f, 1.f);
  const Angle rotationThreshold = distanceFactor * p.rotationThresholdRange.max + (1.f - distanceFactor) * p.rotationThresholdRange.min;
  const float translationXThreshold = distanceFactor * p.translationXThresholdRange.max + (1.f - distanceFactor) * p.translationXThresholdRange.min;
  const float translationYThreshold = distanceFactor * p.translationYThresholdRange.max + (1.f - distanceFactor) * p.translationYThresholdRange.min;
  return Pose2f(rotationThreshold, translationXThreshold, translationYThreshold);
}

//this method checks, if the goalkeeper should stop
bool Goalkeeper::shouldStop(const Pose2f& target) const
{
  const Pose2f error = target.inverse() * theRobotPose;
  return std::abs(error.rotation) < p.shouldStopRotation &&
         std::abs(error.translation.x()) < p.shouldStopTranslation.x() &&
         std::abs(error.translation.y()) < p.shouldStopTranslation.y();
}

//calculates the position of the goalkeeper in the global coordinate system
Pose2f Goalkeeper::calcGlobalBobPose() const
{
  const Vector2f gloPos = theFieldBall.recentBallPositionOnField(p.timeOutBallLastSeen, p.timeOutBallDisappear);

  Vector2f gloBall = gloPos;
  gloBall.x() = std::max(gloPos.x(), theFieldDimensions.xPosOwnGoalLine + p.distanceToGoalLineForMinimumXValue);
  gloBall.y() = std::abs(gloBall.y()) < theFieldDimensions.yPosLeftFieldBorder ? gloBall.y() : sgn(gloBall.y()) * theFieldDimensions.yPosLeftFieldBorder;

  ASSERT(gloBall.allFinite());

  const Vector2f leftPost(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftGoal);
  const Vector2f rightPost(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightGoal);

  const Vector2f ballToLeftPost = leftPost - gloBall;
  const Vector2f ballToRightPost = rightPost - gloBall;

  Angle openingAngle = ballToLeftPost.angleTo(ballToRightPost);
  ASSERT(openingAngle > 0.f);

  const float dist = std::sqrt(sqr(2.f * theBehaviorParameters.genuflectRadius) / (2.f * (1.f - std::cos(openingAngle))));

  Pose2f bobPose;
  bobPose.translation = gloBall + Vector2f(dist, 0.f).rotate(ballToLeftPost.angle() + openingAngle / 2.f);
  bobPose.rotation = (gloBall - bobPose.translation).angle();

  ASSERT(bobPose.isFinite());
  return bobPose;
}

//calculates the position of the goalkeeper on the line between ball and goal
Pose2f Goalkeeper::calcGlobalPoseOnBobLine() const
{
  const Pose2f bobPose(calcGlobalBobPose());
  const Geometry::Line bobLine(bobPose);
  const Vector2f goalPost(theFieldDimensions.xPosOwnGoalLine, bobPose.rotation > 0 ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);
  const Angle rotation(bobPose.rotation > 0 ? p.positionAngle : -p.positionAngle);
  const Geometry::Line postLine(Pose2f(Angle(bobPose.rotation + rotation).normalize(), goalPost));
  Vector2f intersection;
  VERIFY(Geometry::getIntersectionOfLines(bobLine, postLine, intersection));
  return Pose2f(bobPose.rotation, intersection);
}

//calculates the position of the goalkeeper on the line between ball and goal and cuts it with the goal line
Pose2f Goalkeeper::calcGlobalPoseOnBobLineAndCut() const
{
  const Pose2f pose(calcGlobalPoseOnBobLine());
  ASSERT(pose.isFinite());
  const float maxValueX = theFieldDimensions.xPosOwnGoalArea + p.goalieBaseLine;
  Vector2f intersection(pose.translation);
  if(pose.translation.x() > maxValueX)
    VERIFY(Geometry::getIntersectionOfLines(Geometry::Line(pose), Geometry::Line(Pose2f(90_deg, maxValueX, 0.f)), intersection));
  return Pose2f(pose.rotation, intersection);
}
