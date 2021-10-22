/**
 * @file LibPositionProvider.cpp
 * @author Martin Kroker
 * @author Andreas Stolpmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "LibPositionProvider.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(LibPositionProvider, behaviorControl);

void LibPositionProvider::update(LibPosition& libPosition)
{
  DECLARE_DEBUG_DRAWING("behavior:LibPositionProvider:obstacleAtMyPosition", "drawingOnField");

  libPosition.distanceToOwnGoalGreaterThan = [this](float distance) -> bool
  {
    return distanceToOwnGoalGreaterThan(distance);
  };
  libPosition.isInOwnGoalArea = [this](const Vector2f& position)
  {
    return isInOwnGoalArea(position);
  };
  libPosition.isNearOwnGoalArea = [this](const Vector2f& position, float toleranceX, float toleranceY) -> bool
  {
    return isNearOwnGoalArea(position, toleranceX, toleranceY);
  };
  libPosition.isInOwnPenaltyArea = [this](const Vector2f& position) -> bool
  {
    return isInOwnPenaltyArea(position);
  };
  libPosition.isNearOwnPenaltyArea = [this](const Vector2f& position, float toleranceX, float toleranceY) -> bool
  {
    return isNearOwnPenaltyArea(position, toleranceX, toleranceY);
  };
  libPosition.isInOpponentPenaltyArea = [this](const Vector2f& position) -> bool
  {
    return isInOpponentPenaltyArea(position);
  };
  libPosition.isNearOpponentPenaltyArea = [this](const Vector2f& position, float toleranceX, float toleranceY) -> bool
  {
    return isNearOpponentPenaltyArea(position, toleranceX, toleranceY);
  };
  libPosition.isOutSideGoalFrame = [this](const Vector2f& position, const float offset) -> bool
  {
    return isOutSideGoalFrame(position, offset);
  };
  libPosition.getObstacleAtMyPositionCircle = [this](const Vector2f& pos) -> Geometry::Circle
  {
    return getObstacleAtMyPositionCircle(pos);
  };
}

bool LibPositionProvider::distanceToOwnGoalGreaterThan(float distance) const
{
  const Vector2f midPoint(theFieldDimensions.xPosOwnGroundLine, 0.f);
  float distanceToGoal = (theRobotPose.translation - midPoint).norm();
  return distanceToGoal > distance;
}

bool LibPositionProvider::isNearOwnGoalArea(const Vector2f& position, float toleranceX, float toleranceY) const
{
  return std::abs(position.y()) <= theFieldDimensions.yPosLeftGoalArea + toleranceY &&
         position.x() <= theFieldDimensions.xPosOwnGoalArea + toleranceX;
}

bool LibPositionProvider::isInOwnGoalArea(const Vector2f& position) const
{
  return isNearOwnGoalArea(position, 0.f, 0.f);
}

bool LibPositionProvider::isNearOwnPenaltyArea(const Vector2f& position, float toleranceX, float toleranceY) const
{
  return std::abs(position.y()) <= theFieldDimensions.yPosLeftPenaltyArea + toleranceY &&
         position.x() <= theFieldDimensions.xPosOwnPenaltyArea + toleranceX;
}

bool LibPositionProvider::isInOwnPenaltyArea(const Vector2f& position) const
{
  return isNearOwnPenaltyArea(position, 0.f, 0.f);
}

bool LibPositionProvider::isNearOpponentPenaltyArea(const Vector2f& position, float toleranceX, float toleranceY) const
{
  return std::abs(position.y()) <= theFieldDimensions.yPosLeftPenaltyArea + toleranceY &&
         position.x() >= theFieldDimensions.xPosOpponentPenaltyArea - toleranceX;
}

bool LibPositionProvider::isInOpponentPenaltyArea(const Vector2f& position) const
{
  return isNearOpponentPenaltyArea(position, 0.f, 0.f);
}

bool LibPositionProvider::isOutSideGoalFrame(const Vector2f& position, const float offset) const
{
  return position.x() - theFieldDimensions.xPosOwnGroundLine > offset || std::abs(position.y()) - theFieldDimensions.yPosLeftGoal > offset;
}

Geometry::Circle LibPositionProvider::getObstacleAtMyPositionCircle(const Vector2f& position)
{
  CIRCLE("behavior:LibPositionProvider:obstacleAtMyPosition", lastCircle.center.x(), lastCircle.center.y(), lastCircle.radius, 20, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA::black);
  const Vector2f posRel(theRobotPose.inversePose * position);
  for(const Obstacle& obstacle : theObstacleModel.obstacles)
    if((obstacle.center - posRel).squaredNorm() < sqr(positionOffsetIfOccupied))
      return lastCircle = Geometry::Circle(theRobotPose * obstacle.center, positionOffsetIfOccupied);

  if((lastCircle.center - position).squaredNorm() < sqr(lastCircle.radius) &&
     std::abs((theRobotPose.inversePose * lastCircle.center).angle()) > deleteObstacleCircleRange)
    return lastCircle;

  lastCircle.radius = 0.f;
  return lastCircle;
}
