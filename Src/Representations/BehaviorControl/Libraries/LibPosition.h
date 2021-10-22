/**
 * @file LibPosition.h
 *
 * This file defines a representation that provides helpful methods for positions
 *
 * @author Daniel Krause
 */

#pragma once
#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibPosition,
{
  /**
   * @brief Determines whether the distance of the robot to the middle point
   * of the goal is greater than the specified value.
   * @param distance Distance to check.
   * @return Whether the robot is further away from its goal than the
   * specified distance.
   */
  FUNCTION(bool(float distance)) distanceToOwnGoalGreaterThan;

  /**
   * Determines whether the given position is in the own goal area given a tolerance value.
   * Besides the tolerance, the field line width is already taken into account.
   * Thus a position on a line which belongs to the goal area will be considered
   * inside the area.
   * @param position Absolute position on the field
   * @param toleranceX tolerance on x axis.
   * @param toleranceY tolerance on y axis.
   * @return Whether the position is inside the own goal area.
   */
  FUNCTION(bool(const Vector2f& position, float toleranceX, float toleranceY)) isNearOwnGoalArea;

  /**
   * Checks whether the given position is in own goal area using no tolerance.
   * @param position Absolute position on the field
   * @return Whether the position is inside the own goal area.
   */
  FUNCTION(bool(const Vector2f& position)) isInOwnGoalArea;

  /**
   * Determines whether the given position is in the own penalty area given a tolerance value.
   * Besides the tolerance, the field line width is already taken into account.
   * Thus a position on a line which belongs to the penalty area will be considered
   * inside the area.
   * @param position Absolute position on the field
   * @param toleranceX tolerance on x axis in mm.
   * @param toleranceY tolerance on y axis in mm.
   * @return Whether the position is inside the own penalty area or the tolerance area around it.
   */
  FUNCTION(bool(const Vector2f& position, float toleranceX, float toleranceY)) isNearOwnPenaltyArea;

  /**
   * Checks whether the given position is in the own penalty area using no tolerance.
   * @param position Absolute position on the field
   * @return Whether the position is inside the own penalty area.
   */
  FUNCTION(bool(const Vector2f& position)) isInOwnPenaltyArea;

  /**
   * Determines whether the given position is in the opponents penalty area given a tolerance value.
   * Besides the tolerance, the field line width is already taken into account.
   * Thus a position on a line which belongs to the penalty area will be considered
   * inside the area.
   * @param position Absolute position on the field
   * @param toleranceX tolerance on x axis in mm.
   * @param toleranceY tolerance on y axis in mm.
   * @return Whether the position is inside the opponents penalty area or the tolerance area around it.
   */
  FUNCTION(bool(const Vector2f& position, float toleranceX, float toleranceY)) isNearOpponentPenaltyArea;

  /**
   * Checks whether the given position is in the opponents penalty area using no tolerance.
   * @param position Absolute position on the field
   * @return Whether the position is inside the opponents penalty area.
   */
  FUNCTION(bool(const Vector2f& position)) isInOpponentPenaltyArea;

  /**
   * Checks whether the given position is outside + offset the own goal.
   * @param position Absolute position on the field
   * @return Whether the position far enough in the field
   */
  FUNCTION(bool(const Vector2f& position, const float offset)) isOutSideGoalFrame;

  /**
   * Calculates the circle around an obstacle which occupies my target position.
   * @param position The potential target position on the field
   * @return The circle with the obstacle as center or radius 0 if none
   */
  FUNCTION(Geometry::Circle(const Vector2f& position)) getObstacleAtMyPositionCircle,
});
