#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(Obstacle,
{ // Definition of an obstacle
  ENUM(Type, /**< The type of an obstacle. Any changes have huge impact on ObstacleModelProvider.cpp */
  {,
    goalpost,
    unknown,
    someRobot,
    opponent,
    teammate,
    fallenSomeRobot,
    fallenOpponent,
    fallenTeammate,
  });

  static constexpr float getRobotDepth() {return 80.f;};
  static void fusion2D(Obstacle& one, const Obstacle& other);
  void setLeftRight(const float radius);

  Obstacle() = default;
  Obstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right, const Vector2f& velocity, Type type);
  Obstacle(const Matrix2f& covariance, const Vector2f& center, Type type = Type::teammate),

  (Matrix2f) covariance,                  /**< Covariance matrix of an obstacle */
  (Vector2f) center,                      /**< Center point of an Obstacle */
  (Vector2f) left,                        /**< Left point of an Obstacle */
  (Vector2f) right,                       /**< Right point of an Obstacle */
  (Vector2f)(Vector2f::Zero()) velocity, /**< Determined via extended kalman filter (mm per ms) */
  (Type) type,                            /**< See enumeration 'Type' above */
});
