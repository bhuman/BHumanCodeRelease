/**
 * @file LibWalkProvider.cpp
 *
 * @author Andreas Stolpmann
 */

#include "LibWalkProvider.h"

MAKE_MODULE(LibWalkProvider);

void LibWalkProvider::update(LibWalk& libWalk)
{
  DECLARE_DEBUG_DRAWING("behavior:LibWalkProvider:obstacles", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:LibWalkProvider:angles", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:LibWalkProvider:originalPose", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:LibWalkProvider:newPose", "drawingOnField");
  DECLARE_DEBUG_RESPONSE("behavior:LibWalkProvider:newOffset");

  if(!activeLastFrame)
    lastAvoidanceAngleOffset = 0.f;
  activeLastFrame = false;

  libWalk.calcObstacleAvoidance = [this](const Pose2f& originalTarget, bool rough, bool disableObstacleAvoidance, bool toBall) -> MotionRequest::ObstacleAvoidance
  {
    return calcObstacleAvoidance(originalTarget, rough, disableObstacleAvoidance, toBall);
  };
}

MotionRequest::ObstacleAvoidance LibWalkProvider::calcObstacleAvoidance(const Pose2f& originalTarget, bool rough, bool disableObstacleAvoidance, bool toBall)
{
  activeLastFrame = true;

  Pose2f target = originalTarget;

  const Vector2f targetOnField = theRobotPose * target.translation;

  if(target.translation == Vector2f::Zero())
  {
    target.translation = Vector2f(1.f, 0.f).rotate(target.rotation);
    lastAvoidanceAngleOffset = 0.f;
  }

  calculateObstacles(target, targetOnField, rough, disableObstacleAvoidance, toBall);

  // Calculate avoidance as normal
  const Angle targetAngle = target.translation.angle();
  Angle newAvoidanceAngleOffset = lastAvoidanceAngleOffset;
  calculateAvoidanceAngleOffset(newAvoidanceAngleOffset, targetAngle, targetOnField);

  if(rough)
  {
    Angle avoidanceAngle = newAvoidanceAngleOffset + targetAngle;
    if(std::abs(targetAngle) < pi_2)
    {
      if(avoidanceAngle > pi_2)
        avoidanceAngle = pi_2;
      else if(avoidanceAngle < -pi_2)
        avoidanceAngle = -pi_2;
    }
    else
    {
      if(avoidanceAngle > pi)
        avoidanceAngle = pi;
      else if(avoidanceAngle < -pi)
        avoidanceAngle = -pi;
    }
    newAvoidanceAngleOffset = avoidanceAngle - targetAngle;
  }

  // If rough equals true, but an avoidance was calculated, we need to recalculate the avoidance
  // Otherwise the robot would be allowed to walk through the ball to avoid an obstacle
  if(rough && newAvoidanceAngleOffset != 0.f)
  {
    newAvoidanceAngleOffset = lastAvoidanceAngleOffset;
    deactivateRough();
    calculateAvoidanceAngleOffset(newAvoidanceAngleOffset, targetAngle, targetOnField);
  }

  lastAvoidanceAngleOffset = newAvoidanceAngleOffset;

  MotionRequest::ObstacleAvoidance obstacleAvoidance;
  if(lastAvoidanceAngleOffset != 0.f)
  {
    obstacleAvoidance.path.emplace_back();
    obstacleAvoidance.path.back().obstacle = Geometry::Circle(target.translation.rotated(lastAvoidanceAngleOffset), 0.f);
    obstacleAvoidance.path.back().clockwise = lastAvoidanceAngleOffset > 0.f;
  }

  COMPLEX_DRAWING("behavior:LibWalkProvider:angles")
  {
    const Angle angleLeft = getNextFreeAngle(targetAngle, true);
    const Angle angleRight = getNextFreeAngle(targetAngle, false);
    const Angle angleOffsetLeft = Angle::normalize(angleLeft - targetAngle);
    const Angle angleOffsetRight = Angle::normalize(angleRight - targetAngle);
    const float ratingLeft = std::abs(angleOffsetLeft) + (sgn(lastAvoidanceAngleOffset) == sgn(angleOffsetLeft) ? -pi_2 : 0.f);
    const float ratingRight = std::abs(angleOffsetRight) + (sgn(lastAvoidanceAngleOffset) == sgn(angleOffsetRight) ? -pi_2 : 0.f);
    Vector2f left(2000.f, 0.f);
    Vector2f right(2000.f, 0.f);
    left.rotate(angleLeft);
    right.rotate(angleRight);
    left = theRobotPose * left;
    right = theRobotPose * right;
    LINE("behavior:LibWalkProvider:angles", theRobotPose.translation.x(), theRobotPose.translation.y(), left.x(), left.y(), 10, Drawings::solidPen, ratingLeft < ratingRight ? ColorRGBA::blue : ColorRGBA::green);
    LINE("behavior:LibWalkProvider:angles", theRobotPose.translation.x(), theRobotPose.translation.y(), right.x(), right.y(), 10, Drawings::solidPen, ratingLeft < ratingRight ? ColorRGBA::red : ColorRGBA::blue);
  }

  DRAW_ROBOT_POSE("behavior:LibWalkProvider:originalPose", (theRobotPose + originalTarget), ColorRGBA::blue);
  DRAW_ROBOT_POSE("behavior:LibWalkProvider:newPose", (theRobotPose + target), ColorRGBA::green);

  return obstacleAvoidance;
}

void LibWalkProvider::calculateObstacles(const Pose2f& target, const Vector2f& targetOnField, const bool rough, const bool disableObstacleAvoidance, bool toBall)
{
  obstacles.clear();

  if(!disableObstacleAvoidance)
  {
    // obstacles from ObstacleModel
    for(const auto& obstacle : theObstacleModel.obstacles)
      if(obstacle.center.squaredNorm() < sqr(obstacleAvoidanceDistance))
        addObstacle(obstacle.center, obstacle.left, obstacle.right, obstacleAvoidanceMinRadius, obstacleAvoidanceMaxRadius, obstacleAvoidanceDistance);
  }

  // the field border is an obstacle, except if the target is not inside the field (in case theRobotPose is wrong and the ball deemed outside the field)
  if(!theGameState.isPlaying() || !toBall ||
     (std::abs(targetOnField.x()) < theFieldDimensions.xPosOpponentGoalLine
      && std::abs(targetOnField.y()) < theFieldDimensions.yPosLeftTouchline))
  {
    if(theRobotPose.translation.y() > leftOpponentCorner.y() - fieldBorderAvoidanceDistance)
    {
      Vector2f obstaclePos(theRobotPose.translation.x(), leftOpponentCorner.y());
      if(obstaclePos.y() < theRobotPose.translation.y() + 10.f)
        obstaclePos.y() = theRobotPose.translation.y() + 10.f;
      obstaclePos = theRobotPose.inverse() * obstaclePos;
      addObstacle(obstaclePos, 300.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);
    }
    if(theRobotPose.translation.y() < rightOpponentCorner.y() + fieldBorderAvoidanceDistance)
    {
      Vector2f obstaclePos(theRobotPose.translation.x(), rightOpponentCorner.y());
      if(obstaclePos.y() > theRobotPose.translation.y() - 10.f)
        obstaclePos.y() = theRobotPose.translation.y() - 10.f;
      obstaclePos = theRobotPose.inverse() * obstaclePos;
      addObstacle(obstaclePos, 300.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);
    }
    if(theRobotPose.translation.x() > leftOpponentCorner.x() - fieldBorderAvoidanceDistance)
    {
      Vector2f obstaclePos(leftOpponentCorner.x(), theRobotPose.translation.y());
      if(obstaclePos.x() < theRobotPose.translation.x() + 10.f)
        obstaclePos.x() = theRobotPose.translation.x() + 10.f;
      obstaclePos = theRobotPose.inverse() * obstaclePos;
      addObstacle(obstaclePos, 300.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);
    }
    if(theRobotPose.translation.x() < leftOwnCorner.x() + fieldBorderAvoidanceDistance)
    {
      Vector2f obstaclePos(leftOwnCorner.x(), theRobotPose.translation.y());
      if(obstaclePos.x() > theRobotPose.translation.x() - 10.f)
        obstaclePos.x() = theRobotPose.translation.x() - 10.f;
      obstaclePos = theRobotPose.inverse() * obstaclePos;
      addObstacle(obstaclePos, 300.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);
    }
  }

  // Goalposts
  const float sqrGoalPostAvoidanceDistance = goalPostAvoidanceDistance * goalPostAvoidanceDistance;
  for(int i = 0; i < 4; ++i)
  {
    if((theRobotPose.translation - goalPosts[i]).squaredNorm() < sqrGoalPostAvoidanceDistance)
      addObstacle(theRobotPose.inverse() * goalPosts[i], theFieldDimensions.goalPostRadius, goalPostAvoidanceMinRadius, goalPostAvoidanceMaxRadius, goalPostAvoidanceDistance);
  }

  // goal triangles are obstacles
  float sqrFieldBorderAvoidanceDistance = sqr(fieldBorderAvoidanceDistance);
  Vector2f closestPointOnLine;
  if(getSqrDistanceToLine(goalPosts[0], Vector2f(1.f, 0.f), 1000.f, theRobotPose.translation, closestPointOnLine) < sqrFieldBorderAvoidanceDistance)
    addObstacle(theRobotPose.inverse() * closestPointOnLine, 200.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);
  if(getSqrDistanceToLine(goalPosts[1], Vector2f(1.f, 0.f), 1000.f, theRobotPose.translation, closestPointOnLine) < sqrFieldBorderAvoidanceDistance)
    addObstacle(theRobotPose.inverse() * closestPointOnLine, 200.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);
  if(getSqrDistanceToLine(goalPosts[2], Vector2f(-1.f, 0.f), 1000.f, theRobotPose.translation, closestPointOnLine) < sqrFieldBorderAvoidanceDistance)
    addObstacle(theRobotPose.inverse() * closestPointOnLine, 200.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);
  if(getSqrDistanceToLine(goalPosts[3], Vector2f(-1.f, 0.f), 1000.f, theRobotPose.translation, closestPointOnLine) < sqrFieldBorderAvoidanceDistance)
    addObstacle(theRobotPose.inverse() * closestPointOnLine, 200.f, fieldBorderAvoidanceMinRadius, fieldBorderAvoidanceMaxRadius, fieldBorderAvoidanceDistance);

  // Ball
  if(!rough)
    static_cast<void>(applyNotRough()); // return value does not matter

  //////
  if(rough)
  {
    const float targetSqrDistance = target.translation.squaredNorm();
    for(Obstacle& o : obstacles)
    {
      const float t = (o.center.dot(target.translation)) / targetSqrDistance;
      if(t < 0.f || t > 1.f || (o.left.squaredNorm() > targetSqrDistance && o.right.squaredNorm() > targetSqrDistance))
        o.active = false;
    }
  }
  /// the penalty area obstacles are added after the rough check, so that they can't be deactivated
  if(theIllegalAreas.illegal & bit(IllegalAreas::ownPenaltyArea))
  {
    if(theLibPosition.isNearOwnPenaltyArea(theRobotPose.translation, penaltyAreaAvoidanceDistance, penaltyAreaAvoidanceDistance))
    {
      // compute "obstacle" pos
      const Vector2f supportPoint(theFieldDimensions.xPosOwnGoalArea, 0.f);
      Vector2f obstacleDir = supportPoint - theRobotPose.translation;
      obstacleDir.normalize(penaltyAreaAvoidanceDistance);
      Vector2f obstaclePos = theRobotPose.translation + obstacleDir;
      if(theLibPosition.isInOwnPenaltyArea(obstaclePos))
      {
        float xOff = (leftPenCorner.x() - obstaclePos.x());
        float yOff = (obstaclePos.y() > 0.f ? (leftPenCorner.y() - obstaclePos.y()) : (obstaclePos.y() - rightPenCorner.y()));
        float a = std::min(xOff, yOff);
        obstacleDir *= std::max(penaltyAreaAvoidanceDistance - a, 1.f) * (1.f / penaltyAreaAvoidanceDistance);
        if(yOff < xOff)
          obstacleDir = Vector2f(0.f, obstaclePos.y() > 0.f ? -obstacleDir.norm() : obstacleDir.norm());
        else
          obstacleDir = Vector2f(-obstacleDir.norm(), 0.f);
        obstaclePos = theRobotPose.translation + obstacleDir;
      }
      obstaclePos = theRobotPose.inverse() * obstaclePos;
      addObstacle(obstaclePos, 300.f, penaltyAreaAvoidanceMinRadius, penaltyAreaAvoidanceMaxRadius, penaltyAreaAvoidanceDistance);
    }
  }
  if(theIllegalAreas.illegal & bit(IllegalAreas::opponentPenaltyArea))
  {
    if(theLibPosition.isNearOpponentPenaltyArea(theRobotPose.translation, penaltyAreaAvoidanceDistance, penaltyAreaAvoidanceDistance))
    {
      // compute "obstacle" pos
      const Vector2f supportPoint(theFieldDimensions.xPosOpponentGoalArea, 0.f);
      Vector2f obstacleDir = supportPoint - theRobotPose.translation;
      obstacleDir.normalize(penaltyAreaAvoidanceDistance);
      Vector2f obstaclePos = theRobotPose.translation + obstacleDir;
      if(theLibPosition.isInOpponentPenaltyArea(obstaclePos))
      {
        float xOff = obstaclePos.x() - theFieldDimensions.xPosOpponentPenaltyArea;
        float yOff = (obstaclePos.y() > 0.f ? (leftPenCorner.y() - obstaclePos.y()) : (obstaclePos.y() - rightPenCorner.y()));
        float a = std::min(xOff, yOff);
        obstacleDir *= std::max(penaltyAreaAvoidanceDistance - a, 1.f) * (1.f / penaltyAreaAvoidanceDistance);
        if(yOff < xOff)
          obstacleDir = Vector2f(0.f, obstaclePos.y() > 0.f ? -obstacleDir.norm() : obstacleDir.norm());
        else
          obstacleDir = Vector2f(obstacleDir.norm(), 0.f);
        obstaclePos = theRobotPose.translation + obstacleDir;
      }
      obstaclePos = theRobotPose.inverse() * obstaclePos;
      addObstacle(obstaclePos, 300.f, penaltyAreaAvoidanceMinRadius, penaltyAreaAvoidanceMaxRadius, penaltyAreaAvoidanceDistance);
    }
  }

  //////////////

  COMPLEX_DRAWING("behavior:LibWalkProvider:obstacles")
  {
    for(const Obstacle& o : obstacles)
    {
      const ColorRGBA color = o.active ? ColorRGBA::red : ColorRGBA::green;
      auto left = theRobotPose * o.left;
      auto right = theRobotPose * o.right;
      auto outerLeft = theRobotPose * o.outerLeft;
      auto outerRight = theRobotPose * o.outerRight;
      LINE("behavior:LibWalkProvider:obstacles", left.x(), left.y(), right.x(), right.y(), 20, Drawings::solidPen, color);
      LINE("behavior:LibWalkProvider:obstacles", left.x(), left.y(), outerLeft.x(), outerLeft.y(), 20, Drawings::solidPen, color);
      LINE("behavior:LibWalkProvider:obstacles", right.x(), right.y(), outerRight.x(), outerRight.y(), 20, Drawings::solidPen, color);
    }
  }

  obstaclesSortedLeft = obstacles;
  obstaclesSortedRight = obstacles;

  std::sort(obstaclesSortedLeft.begin(), obstaclesSortedLeft.end(),
            [](const Obstacle& o1, const Obstacle& o2) -> bool
  {
    return o1.angleRight < o2.angleRight;
  });

  std::sort(obstaclesSortedRight.begin(), obstaclesSortedRight.end(),
            [](const Obstacle& o1, const Obstacle& o2) -> bool
  {
    return o1.angleLeft < o2.angleLeft;
  });
}

Angle LibWalkProvider::getNextFreeAngle(const Angle baseAngle, const bool ccw)
{
  Angle angle = baseAngle;
  if(ccw)
  {
    for(const Obstacle& o : obstaclesSortedLeft)
      if(o.active && ((o.angleRight <= o.angleLeft && angle > o.angleRight && angle < o.angleLeft)
                      || (o.angleRight > o.angleLeft && (angle > o.angleRight || angle < o.angleLeft))))
        angle = o.angleLeft;
  }
  else
  {
    for(const Obstacle& o : obstaclesSortedRight)
      if(o.active && ((o.angleRight <= o.angleLeft && angle > o.angleRight && angle < o.angleLeft)
                      || (o.angleRight > o.angleLeft && (angle > o.angleRight || angle < o.angleLeft))))
        angle = o.angleRight;
  }
  return angle;
}

void LibWalkProvider::addObstacle(const Vector2f& center, const float radius,
                                  const float minAvoidanceRadius, const float maxAvoidanceRadius, const float avoidanceDistance)
{
  Vector2f left = center;
  Vector2f right = center;
  left.normalize(radius);
  right.normalize(radius);
  left.rotateLeft();
  right.rotateRight();
  left += center;
  right += center;
  addObstacle(center, left, right, minAvoidanceRadius, maxAvoidanceRadius, avoidanceDistance);
}

void LibWalkProvider::addObstacle(const Vector2f& center, const Vector2f& left, const Vector2f& right,
                                  const float minAvoidanceRadius, const float maxAvoidanceRadius, const float avoidanceDistance)
{
  const float radius = minAvoidanceRadius + (maxAvoidanceRadius - minAvoidanceRadius) * (avoidanceDistance - center.norm()) / avoidanceDistance;

  const Vector2f outerLeft = (left + Vector2f(-left.y(), left.x()).normalize(radius));
  const Vector2f outerRight = (right + Vector2f(right.y(), -right.x()).normalize(radius));
  const Angle angleLeft = outerLeft.angle();
  const Angle angleRight = outerRight.angle();

  obstacles.emplace_back(Obstacle(angleLeft, angleRight, center, left, right, outerLeft, outerRight));
}

float LibWalkProvider::getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point, Vector2f& orthogonalProjection)
{
  float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  if(l < 0)
    l = 0;
  if(l > length)
    l = length;
  orthogonalProjection = base + dir * l;
  return (orthogonalProjection - point).squaredNorm();
}

void LibWalkProvider::deactivateRough()
{
  for(Obstacle& o : obstacles)
    o.active = true;
  if(applyNotRough())
  {
    COMPLEX_DRAWING("behavior:LibWalkProvider:obstacles")
    {
      const Obstacle& o = obstacles.back();
      const ColorRGBA color = o.active ? ColorRGBA::red : ColorRGBA::green;
      auto left = theRobotPose * o.left;
      auto right = theRobotPose * o.right;
      auto outerLeft = theRobotPose * o.outerLeft;
      auto outerRight = theRobotPose * o.outerRight;
      LINE("behavior:LibWalkProvider:obstacles", left.x(), left.y(), right.x(), right.y(), 20, Drawings::solidPen, color);
      LINE("behavior:LibWalkProvider:obstacles", left.x(), left.y(), outerLeft.x(), outerLeft.y(), 20, Drawings::solidPen, color);
      LINE("behavior:LibWalkProvider:obstacles", right.x(), right.y(), outerRight.x(), outerRight.y(), 20, Drawings::solidPen, color);
    }
  }

  obstaclesSortedLeft = obstacles;
  obstaclesSortedRight = obstacles;

  std::sort(obstaclesSortedLeft.begin(), obstaclesSortedLeft.end(),
            [](const Obstacle& o1, const Obstacle& o2) -> bool
  {
    return o1.angleRight < o2.angleRight;
  });

  std::sort(obstaclesSortedRight.begin(), obstaclesSortedRight.end(),
            [](const Obstacle& o1, const Obstacle& o2) -> bool
  {
    return o1.angleLeft < o2.angleLeft;
  });
}

bool LibWalkProvider::applyNotRough()
{
  // Ball
  if(theGameState.isPlaying()
     && (theFieldBall.timeSinceBallWasSeen < ballTimeout || theFieldBall.teammatesBallIsValid))
  {
    const Vector2f& ballPosition = theFieldBall.recentBallPositionRelative(ballTimeout);
    if(ballPosition.squaredNorm() < sqr(ballAvoidanceDistance))
    {
      addObstacle(ballPosition, theBallSpecification.radius, ballAvoidanceMinRadius, ballAvoidanceMaxRadius, ballAvoidanceDistance);
      return true;
    }
  }
  return false;
}

void LibWalkProvider::calculateAvoidanceAngleOffset(Angle& newAvoidanceAngleOffset, const Angle targetAngle, const Vector2f& targetOnField)
{
  const Angle angleLeft = getNextFreeAngle(targetAngle, true);
  const Angle angleRight = getNextFreeAngle(targetAngle, false);

  const Angle angleOffsetLeft = Angle::normalize(angleLeft - targetAngle);
  const Angle angleOffsetRight = Angle::normalize(angleRight - targetAngle);

  const float ratingLeft = std::abs(angleOffsetLeft) + (sgn(lastAvoidanceAngleOffset) == sgn(angleOffsetLeft) ? -pi_2 : 0.f);
  const float ratingRight = std::abs(angleOffsetRight) + (sgn(lastAvoidanceAngleOffset) == sgn(angleOffsetRight) ? -pi_2 : 0.f);

  newAvoidanceAngleOffset = ratingLeft < ratingRight ? angleOffsetLeft : angleOffsetRight;

  /// Hack so the robot can always walk to the ball if it is on the goal line
  if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea
     && targetOnField.x() < theFieldDimensions.xPosOwnGoalLine && std::abs(targetOnField.y()) < theFieldDimensions.yPosLeftGoal)
  {
    if(targetOnField.y() > theFieldDimensions.yPosLeftGoal - 250.f && theRobotPose.translation.y() > targetOnField.y())
      newAvoidanceAngleOffset = angleOffsetLeft;
    if(targetOnField.y() < theFieldDimensions.yPosRightGoal + 250.f && theRobotPose.translation.y() < targetOnField.y())
      newAvoidanceAngleOffset = angleOffsetRight;
  }
}
