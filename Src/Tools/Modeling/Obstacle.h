#pragma once

#include "Streaming/AutoStreamable.h"
#include "Debugging/Debugging.h"
#include "Math/Eigen.h"

STREAMABLE(Obstacle,
{
  // The different obstacle types
  ENUM(Type, /**< The type of an obstacle. Current code assumes the "fallen" elements to be at the end of the list */*
  {,
    unknown,
    someRobot,
    opponent,
    teammate,
    fallenSomeRobot,
    fallenOpponent,
    fallenTeammate,
  });

  static void fusion2D(Obstacle& one, const Obstacle& other);
  void setLeftRight(const float radius);
  bool isTeammate() const;
  bool isOpponent() const;
  bool isSomeRobot() const;
  bool isUnknown() const;

  Obstacle() = default;
  Obstacle(const Matrix2f& covariance, const Vector2f& center, const Vector2f& left, const Vector2f& right, const Vector2f& velocity,
           unsigned int lastSeen, Type type);
  Obstacle(const Matrix2f& covariance, const Vector2f& center, float radius, unsigned int lastSeen = 0, Type type = Type::teammate),

  (Matrix2f) covariance,                  /**< Covariance matrix of an obstacle */
  (Vector2f) center,                      /**< Center point of an obstacle */
  (Vector2f) left,                        /**< Left point of an obstacle */
  (Vector2f) right,                       /**< Right point of an obstacle */
  (Vector2f)(Vector2f::Zero()) velocity,  /**< Determined via extended Kalman filter (mm per ms) */
  (unsigned int) lastSeen,                /**< Timestamp of last measurement */
  (Type) type,                            /**< See enumeration 'Type' above */
});
