/**
 * @file Defender.cpp
 *
 * This file implements the defender role.
 *
 * @author Jesse Richter-Klug
 * @author Arne Hasselbring
 */

#include "Defender.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/Modify.h"
#include "Debugging/Annotation.h"

void Defender::preProcess()
{
  MODIFY("parameters:behavior:Defender", p);
  DECLARE_DEBUG_DRAWING("behavior:Defender:position", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Defender:coverage", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Defender:isLeft", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Defender:bobLines", "drawingOnField");
}

Pose2f Defender::position(Side side, const Pose2f& basePose, const std::vector<Vector2f>&, const Agents& teammates)
{
  if(theGameState.isGoalKick() && theGameState.isForOwnTeam())
    return basePose;

  goalkeeper = teammates.byPosition(Tactic::Position::goalkeeper);
  if(side == unspecified || side == center)
  {
    otherDefender = nullptr;
    side = calcDefenderRoyaleIsLeft(theRobotPose.translation.y() >= 0.f) ? left : right;
  }
  else if(side == left)
    otherDefender = teammates.byPosition(Tactic::Position::defenderR);
  else
    otherDefender = teammates.byPosition(Tactic::Position::defenderL);

  Vector2f targetOnField = calcDefenderRoyalePosition(side == left);
  if(theFieldDimensions.clipToField(targetOnField) > 0.f && theFrameInfo.getTimeSince(annotationTimestamp) > p.annotationTime)
  {
    annotationTimestamp = theFrameInfo.time;
    ANNOTATION("behavior:role:Defender", "Clipped defender position into field");
  }

  return Pose2f((theFieldBall.recentBallPositionOnField() - targetOnField).angle(), targetOnField);
}

Pose2f Defender::tolerance() const
{
  const float distanceFactor = mapToRange(theFieldBall.recentBallPositionRelative().norm(), p.ballDistanceInterpolationRange.min, p.ballDistanceInterpolationRange.max, 0.f, 1.f);
  const Angle rotationThreshold = distanceFactor * p.rotationThresholdRange.max + (1.f - distanceFactor) * p.rotationThresholdRange.min;
  const float translationXThreshold = distanceFactor * p.translationXThresholdRange.max + (1.f - distanceFactor) * p.translationXThresholdRange.min;
  const float translationYThreshold = distanceFactor * p.translationYThresholdRange.max + (1.f - distanceFactor) * p.translationYThresholdRange.min;
  return Pose2f(rotationThreshold, translationXThreshold, translationYThreshold);
}

bool Defender::shouldStop(const Pose2f& target) const
{
  const Pose2f error = target.inverse() * theRobotPose;
  return std::abs(error.rotation) < p.shouldStopRotation &&
         std::abs(error.translation.x()) < p.shouldStopTranslation.x() &&
         std::abs(error.translation.y()) < p.shouldStopTranslation.y();
}

Vector2f Defender::calcDefenderRoyalePosition(const bool isLeftDefender) const
{
  const Vector2f ballPositionField(theFieldBall.recentBallPositionOnField());

  const Vector2f leftUpperDefenderGoalArea(theFieldDimensions.xPosOwnGoalArea + p.defenderRoyaleDistanceToGoalArea,
                                           theFieldDimensions.yPosLeftGoalArea + p.defenderRoyaleDistanceToGoalArea);
  const Vector2f rightUpperDefenderGoalArea(theFieldDimensions.xPosOwnGoalArea + p.defenderRoyaleDistanceToGoalArea,
                                            theFieldDimensions.yPosRightGoalArea - p.defenderRoyaleDistanceToGoalArea);

  LINE("behavior:Defender:position", leftUpperDefenderGoalArea.x(), leftUpperDefenderGoalArea.y(), rightUpperDefenderGoalArea.x(), rightUpperDefenderGoalArea.y(), 20, Drawings::dottedPen, ColorRGBA::gray);

  const Vector2f circleBase(theFieldDimensions.xPosOwnGroundLine, 0.f);
  const float minRadius((circleBase - leftUpperDefenderGoalArea).norm());
  const float maxRadius(theFieldDimensions.yPosLeftSideline);

  CIRCLE("behavior:Defender:position", circleBase.x(), circleBase.y(), minRadius, 20, Drawings::dashedPen, ColorRGBA::gray, Drawings::noPen, ColorRGBA::black);
  CIRCLE("behavior:Defender:position", circleBase.x(), circleBase.y(), maxRadius, 20, Drawings::dashedPen, ColorRGBA::gray, Drawings::noPen, ColorRGBA::black);

  const Geometry::Line cutLine(Pose2f(-pi_2, leftUpperDefenderGoalArea));
  Vector2f firstIntersection;
  Vector2f secondIntersection;

  const Angle minAngleMayChangeSide((Vector2f(theFieldDimensions.xPosOwnGroundLine - (theFieldDimensions.xPosOwnGroundLine - theFieldDimensions.xPosOwnPenaltyArea), theFieldDimensions.yPosLeftSideline) - circleBase).angle());
  const Angle minAngleMustChangeSide((Vector2f(theFieldDimensions.xPosOwnPenaltyArea + (theFieldDimensions.xPosOwnGroundLine - theFieldDimensions.xPosOwnPenaltyArea) / 3, theFieldDimensions.yPosLeftSideline) - circleBase).angle());
  const Angle ballAngle((ballPositionField - circleBase).angle());

  const bool single = !otherDefender;

  float radius(minRadius);
  const bool forward = !single && isRoyalePositionForward(radius, isLeftDefender, otherDefender, maxRadius, minRadius, ballPositionField);

  std::vector<Vector2f> intersections;
  std::vector<Geometry::Line> calcLines;

  if((isLeftDefender && ballAngle < minAngleMustChangeSide) || (!isLeftDefender && ballAngle > -minAngleMustChangeSide))
    calcDefenderBObLine(isLeftDefender, radius, calcLines);

  if((isLeftDefender && ballAngle > minAngleMayChangeSide) || (!isLeftDefender && ballAngle < -minAngleMayChangeSide))
    calcDefenderBObLine(!isLeftDefender, radius, calcLines);

  ASSERT(!calcLines.empty());

  CIRCLE("behavior:Defender:position", circleBase.x(), circleBase.y(), radius, 20, Drawings::solidPen, forward ? ColorRGBA::red : ColorRGBA::orange, Drawings::noPen, ColorRGBA::black);

  for(const Geometry::Line& positionLine : calcLines)
  {
    const Geometry::Circle circle(circleBase, radius);
    VERIFY(Geometry::getIntersectionOfLineAndCircle(positionLine, circle, firstIntersection, secondIntersection) == 2);
    Vector2f intersection((firstIntersection - ballPositionField).squaredNorm() < (secondIntersection - ballPositionField).squaredNorm() ? firstIntersection : secondIntersection);

    LINE("behavior:Defender:position", positionLine.base.x(), positionLine.base.y(), positionLine.base.x() + positionLine.direction.x(), positionLine.base.y() + positionLine.direction.y(), 20, Drawings::solidPen, isLeftDefender ? ColorRGBA::yellow : ColorRGBA::blue);
    CROSS("behavior:Defender:position", intersection.x(), intersection.y(), 100, 20, Drawings::solidPen, ColorRGBA::gray);

    if(!forward && intersection.x() > leftUpperDefenderGoalArea.x())
      VERIFY(Geometry::getIntersectionOfLines(positionLine, cutLine, intersection));

    CROSS("behavior:Defender:position", intersection.x(), intersection.y(), 100, 20, Drawings::solidPen, ColorRGBA::gray);

    if(!single)
    {
      if(isLeftDefender)
      {
        const float minYForLeft(theFieldDimensions.yPosRightGoal / 2);
        const Geometry::Line minYForLeftLine(Pose2f(0.f, minYForLeft));
        if(minYForLeft > intersection.y())
        {
          VERIFY(Geometry::getIntersectionOfLines(positionLine, minYForLeftLine, intersection));
          if(leftUpperDefenderGoalArea.x() > intersection.x())
            intersection.x() = leftUpperDefenderGoalArea.x();
        }
      }
      else
      {
        const float maxYForRight(theFieldDimensions.yPosLeftGoal / 2);
        const Geometry::Line maxYForRightLine(Pose2f(0.f, maxYForRight));
        if(maxYForRight < intersection.y())
        {
          VERIFY(Geometry::getIntersectionOfLines(positionLine, maxYForRightLine, intersection));
          if(leftUpperDefenderGoalArea.x() > intersection.x())
            intersection.x() = leftUpperDefenderGoalArea.x();
        }
      }
    }

    CROSS("behavior:Defender:position", intersection.x(), intersection.y(), 100, 20, Drawings::solidPen, isLeftDefender ? ColorRGBA::yellow : ColorRGBA::blue);
    intersections.push_back(intersection);
  }

  ASSERT(!intersections.empty());
  const Vector2f intersection(intersections.size() == 1 || (intersections[0] - theRobotPose.translation).squaredNorm() < (intersections[1] - theRobotPose.translation).squaredNorm()
                              ? intersections[0]
                              : intersections[1]);
  return returnPositionWithDraw(intersection);
}

const Vector2f& Defender::returnPositionWithDraw(const Vector2f& position) const
{
  COMPLEX_DRAWING("behavior:Defender:coverage")
  {
    const Vector2f gloBallPos = theRobotPose * theFieldBall.positionRelative; // todo????

    const Vector2f offset = Vector2f(0.f, 1.f).rotate((gloBallPos - position).angle());

    const float standRange = 80.f;
    const float genuflectRange = 200.f;
    const float jumpRange = 600.f;

    auto drawOffset = [&](const float range, [[maybe_unused]] const ColorRGBA color)
    {
      const Vector2f rangeOffset = offset.normalized(range);
      const Vector2f defenderLeft = position + rangeOffset;
      const Vector2f defenderRight = position - rangeOffset;

      const Geometry::Line leftLine(gloBallPos, defenderLeft - gloBallPos);
      const Geometry::Line rightLine(gloBallPos, defenderRight - gloBallPos);

      const Vector2f bottomLeft(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline);
      const Vector2f topRight(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);

      Vector2f useLeft;
      Vector2f useRight;

      Vector2f intersection1;
      Vector2f intersection2;
      if(!Geometry::getIntersectionPointsOfLineAndRectangle(bottomLeft, topRight, leftLine, intersection1, intersection2))
        return;
      useLeft = (intersection1 - position).squaredNorm() < (intersection1 - gloBallPos).squaredNorm() ? intersection1 : intersection2;

      if(!Geometry::getIntersectionPointsOfLineAndRectangle(bottomLeft, topRight, rightLine, intersection1, intersection2))
        return;
      useRight = (intersection1 - position).squaredNorm() < (intersection1 - gloBallPos).squaredNorm() ? intersection1 : intersection2;

      const Vector2f points[4] = { defenderLeft, defenderRight, useRight, useLeft };
      POLYGON("behavior:Defender:coverage", 4, points, 10, Drawings::noPen, color, Drawings::solidBrush, color);
    };

    const Vector2f points[3] = { gloBallPos, Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal), Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal) };
    POLYGON("behavior:Defender:coverage", 3, points, 10, Drawings::noPen, ColorRGBA(ColorRGBA::red.r, ColorRGBA::red.g, ColorRGBA::red.b, 50), Drawings::solidBrush, ColorRGBA(ColorRGBA::red.r, ColorRGBA::red.g, ColorRGBA::red.b, 50));

    drawOffset(jumpRange, ColorRGBA(ColorRGBA::orange.r, ColorRGBA::orange.g, ColorRGBA::orange.b, 50));
    drawOffset(genuflectRange, ColorRGBA(ColorRGBA::violet.r, ColorRGBA::violet.g, ColorRGBA::violet.b, 100));
    drawOffset(standRange, ColorRGBA(ColorRGBA::green.r, ColorRGBA::green.g, ColorRGBA::green.b, 200));
  }
  return position;
}

bool Defender::isRoyalePositionForward(float& radius, const bool isLeftDefender, const Agent* otherDefender,
                                       const float maxRadius, const float minRadius, const Vector2f& ballPositionField) const
{
  bool forward(ballPositionField.x() > theFieldDimensions.centerCircleRadius);

  const float actualRadius((Pose2f(theFieldDimensions.xPosOwnGroundLine, 0.f).inverse() * theRobotPose.translation).norm());
  const bool actualForward(actualRadius - minRadius > p.minXDiffToBeClearForward);

  const Pose2f secondDefenderPosition(otherDefender->pose);
  const float secondDefenderRadius((Pose2f(theFieldDimensions.xPosOwnGroundLine, 0.f).inverse() * secondDefenderPosition.translation).norm());
  const bool isSecondDefenderForward(secondDefenderRadius - minRadius > p.minXDiffToBeClearForward);

  if(actualForward != isSecondDefenderForward || (!actualForward && !forward))
    forward = actualForward;
  else if((std::abs(theRobotPose.rotation) < pi_4 && std::abs(secondDefenderPosition.rotation) > pi3_4) ||
          (std::abs(theRobotPose.rotation) > pi3_4 && std::abs(secondDefenderPosition.rotation) < pi_4))
    forward = std::abs(theRobotPose.rotation) < pi_4;
  else
    forward = isLeftDefender ? ballPositionField.y() > 0.f : ballPositionField.y() < 0.f;

  if(forward)
    radius = std::min(maxRadius, std::max((Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f) - ballPositionField).norm() - p.defenderRoyaleForwardDistanceToBall, minRadius));
  else
    radius = minRadius;

  return forward;
}

void Defender::calcDefenderBObLine(const bool isLeftDefender, const float radius, std::vector<Geometry::Line>& bobLine) const
{
  const bool isKeeperOnField = goalkeeper && theLibPosition.isInOwnPenaltyArea(goalkeeper->currentPosition);
  const bool single(!otherDefender);

  if(!isKeeperOnField && single)
  {
    bobLine.emplace_back(theGoaliePose.goaliePoseField);
    return;
  }

  const float sign = isLeftDefender ? 1.f : -1.f;

  const Vector2f ballPositionField(theFieldBall.recentBallPositionOnField());
  const Geometry::Line groundLine(Pose2f(pi_2, theFieldDimensions.xPosOwnGroundLine, 0.f));

  const Vector2f outerGoaliePoint = theGoaliePose.goaliePoseField * Vector2f(0.f, sign * theBehaviorParameters.standRadius);
  const Vector2f goalPost(theFieldDimensions.xPosOwnGoalPost, isLeftDefender ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);

  /// that case we want (cover the center between goalie and goal post)
  const Geometry::Line ballOuterGoalieLine(ballPositionField, outerGoaliePoint - ballPositionField);
  const bool isOptimumValid = isLeftDefender == Geometry::isPointLeftOfLine(outerGoaliePoint, ballPositionField, goalPost);
  const Vector2f centerPoint = (goalPost + outerGoaliePoint) / 2.f;
  const Angle positionAngleOptimum = (centerPoint - ballPositionField).angle();
  RAY("behavior:Defender:bobLines", ballPositionField, positionAngleOptimum, 10, Drawings::solidPen, isOptimumValid ? ColorRGBA::white : ColorRGBA::gray);

  const float coverageOffsetToGoalie = sign * theBehaviorParameters.standRadius * p.standOffsetMultiplierToAdjustGoalieDefenderLineDistance;
  const Vector2f outerGoalieOffsetPoint = theGoaliePose.goaliePoseField * Vector2f(0.f, coverageOffsetToGoalie);
  const Vector2f positionOffsetToGoalieAnchorPoint = ballPositionField + (outerGoalieOffsetPoint - theGoaliePose.goaliePoseField.translation);
  const Angle positionOffsetToGoalieAngle = (outerGoalieOffsetPoint - ballPositionField).angle(); //ball position is right here
  RAY("behavior:Defender:bobLines", positionOffsetToGoalieAnchorPoint, positionOffsetToGoalieAngle, 10, Drawings::solidPen, ColorRGBA::green);

  const Vector2f positionGenuOffsetToGoalPostAnchorPoint = ballPositionField - Vector2f(0.f, sign * theBehaviorParameters.genuflectRadius).rotate(theGoaliePose.goaliePoseField.rotation);
  const Angle positionGenuOffsetToGoalPostAngle = (goalPost - ballPositionField).angle();
  RAY("behavior:Defender:bobLines", positionGenuOffsetToGoalPostAnchorPoint, positionGenuOffsetToGoalPostAngle, 10, Drawings::solidPen, ColorRGBA::magenta);

  const Vector2f circleBase(theFieldDimensions.xPosOwnGroundLine, 0.f);
  const Geometry::Circle circle(circleBase, radius);
  auto getComparableIntersectionAngleWithCircle = [&](const Geometry::Line& line, Angle& intersectionAngle)
  {
    Vector2f intersection1, intersection2;
    if(Geometry::getIntersectionOfLineAndCircle(line, circle, intersection1, intersection2) != 2)
      return false;

    if((ballPositionField - intersection1).squaredNorm() > (ballPositionField - intersection2).squaredNorm())
      intersectionAngle = (intersection2 - circleBase).angle() * sign;
    else
      intersectionAngle = (intersection1 - circleBase).angle() * sign;

    return true;
  };

  const Geometry::Line optimumLine(Pose2f(positionAngleOptimum, ballPositionField));
  const Geometry::Line nextToPostLine(Pose2f(positionGenuOffsetToGoalPostAngle, positionGenuOffsetToGoalPostAnchorPoint));
  const Geometry::Line nextToGoalieLine(Pose2f(positionOffsetToGoalieAngle, positionOffsetToGoalieAnchorPoint));

  Angle comparablePositionAngleOptimum, comparablePositionOffsetToGoalieAngle, comparablePositionGenuOffsetToGoalPostAngle;
  VERIFY((!isOptimumValid || getComparableIntersectionAngleWithCircle(optimumLine, comparablePositionAngleOptimum))
         && getComparableIntersectionAngleWithCircle(nextToPostLine, comparablePositionGenuOffsetToGoalPostAngle)
         && getComparableIntersectionAngleWithCircle(nextToGoalieLine, comparablePositionOffsetToGoalieAngle));

  if(isOptimumValid
     && comparablePositionAngleOptimum > comparablePositionOffsetToGoalieAngle
     && comparablePositionAngleOptimum > comparablePositionGenuOffsetToGoalPostAngle)
    bobLine.emplace_back(Pose2f(positionAngleOptimum, ballPositionField));
  else if(comparablePositionOffsetToGoalieAngle > comparablePositionGenuOffsetToGoalPostAngle)
    bobLine.emplace_back(Pose2f(positionOffsetToGoalieAngle, positionOffsetToGoalieAnchorPoint));
  else
    bobLine.emplace_back(Pose2f(positionGenuOffsetToGoalPostAngle, positionGenuOffsetToGoalPostAnchorPoint));
}

bool Defender::calcDefenderRoyaleIsLeft(const bool wasLeft) const
{
  const Vector2f ballPositionOnField(theFieldBall.recentBallPositionOnField());

  DRAW_ROBOT_POSE("behavior:Defender:isLeft", theRobotPose, ColorRGBA::magenta);
  DRAW_TEXT("behavior:Defender:isLeft", 0.f, 0.f, 150, wasLeft ? ColorRGBA::yellow : ColorRGBA::blue, "wasLeft = " << wasLeft);

  LINE("behavior:Defender:isLeft", theFieldDimensions.xPosOwnGroundLine, 0.f, 0.f, theFieldDimensions.yPosLeftSideline, 20, Drawings::solidPen, ColorRGBA::gray);
  LINE("behavior:Defender:isLeft", theFieldDimensions.xPosOwnGroundLine, 0.f, 0.f, theFieldDimensions.yPosRightSideline, 20, Drawings::solidPen, ColorRGBA::gray);

  //the ball is far left -> left
  if(Geometry::isPointLeftOfLine(Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f),
                                 Vector2f(0.f, theFieldDimensions.yPosLeftSideline), ballPositionOnField))
    return returnIsLeftWithDraw(true);
  //the ball ist far right -> right
  if(!Geometry::isPointLeftOfLine(Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f),
                                  Vector2f(0.f, theFieldDimensions.yPosRightSideline), ballPositionOnField))
    return returnIsLeftWithDraw(false);
  //else we stay on our side
  return returnIsLeftWithDraw(wasLeft);
}

bool Defender::returnIsLeftWithDraw(const bool leftArrow) const
{
  COMPLEX_DRAWING("behavior:Defender:isLeft")
  {
    const Vector2f offSet(Vector2f(500, 0.f).rotate(theRobotPose.rotation + (leftArrow ? pi_2 : -pi_2)));
    const Vector2f endPos(theRobotPose.translation + offSet);

    ARROW("behavior:Defender:isLeft", theRobotPose.translation.x(), theRobotPose.translation.y(), endPos.x(), endPos.y(), 20, Drawings::solidPen, leftArrow ? ColorRGBA::yellow : ColorRGBA::blue);
  }
  return leftArrow;
}
