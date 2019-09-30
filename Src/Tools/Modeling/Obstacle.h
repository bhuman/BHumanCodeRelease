#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(Obstacle,
{
  // Definition of an obstacle
  ENUM(Type, /**< The type of an obstacle. Any changes might have an impact on ObstacleModelProvider.cpp/InternalObstacle.cpp <- probably not */ //TODO: check whether order is important
  // but TeamPlayersLocator::setType!!!!!!!!!! ~Arne 13.04.2018
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
  bool isTeammate() const;
  bool isOpponent() const;

  Obstacle() = default;
  Obstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right, const Vector2f& velocity,
           unsigned int lastSeen, Type type);
  Obstacle(const Matrix2f& covariance, const Vector2f& center, unsigned int lastSeen = 0, Type type = Type::teammate),

  (Matrix2f) covariance,                  /**< Covariance matrix of an obstacle */
  (Vector2f) center,                      /**< Center point of an obstacle */
  (Vector2f) left,                        /**< Left point of an obstacle */
  (Vector2f) right,                       /**< Right point of an obstacle */
  (Vector2f)(Vector2f::Zero()) velocity,  /**< Determined via extended Kalman filter (mm per ms) */
  (unsigned int) lastSeen,                /**< Timestamp of last measurement */
  (Type) type,                            /**< See enumeration 'Type' above */
});
