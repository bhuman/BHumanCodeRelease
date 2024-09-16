/**
 * @file LibPositionProvider.cpp
 * @author Martin Kroker
 * @author Andreas Stolpmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "LibPositionProvider.h"

MAKE_MODULE(LibPositionProvider);

void LibPositionProvider::update(LibPosition& libPosition)
{
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
  libPosition.isNearPost = [this](const Pose2f& position) -> std::tuple<bool, bool>
    {
    return isNearPost(position);
  };
}

bool LibPositionProvider::distanceToOwnGoalGreaterThan(float distance) const
{
  const Vector2f midPoint(theFieldDimensions.xPosOwnGoalLine, 0.f);
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
  return position.x() - theFieldDimensions.xPosOwnGoalLine > offset || std::abs(position.y()) - theFieldDimensions.yPosLeftGoal > offset;
}

std::tuple<bool, bool> LibPositionProvider::isNearPost(const Pose2f& position) const
{
  const float jumpDistanceThreshold = 600.f;
  const Angle jumpAngleThresholdMin = 65_deg;
  const Angle jumpAngleThresholdMax = 100_deg;
  const Vector2f leftPost(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
  const Vector2f rightPost(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
  const Vector2f leftRel = position.inverse() * leftPost;
  const Vector2f rightRel = position.inverse() * rightPost;
  const float angleToLeft = leftRel.angle();
  const float angleToRight = rightRel.angle();

  bool isNearLeftPost =
    leftRel.norm() < jumpDistanceThreshold &&
    angleToLeft > jumpAngleThresholdMin &&
    angleToLeft < jumpAngleThresholdMax;
  bool isNearRightPost =
    rightRel.norm() < jumpDistanceThreshold &&
    angleToRight < -jumpAngleThresholdMin &&
    angleToRight > -jumpAngleThresholdMax;

  return std::make_tuple(isNearLeftPost, isNearRightPost);
}
