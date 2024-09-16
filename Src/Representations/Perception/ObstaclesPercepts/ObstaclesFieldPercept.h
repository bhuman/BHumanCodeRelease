/**
 * @file ObstaclesFieldPercept.h
 *
 * This file declares a representation that lists the obstacles that were detected in
 * the current image in robot-relative field coordinates. Only obstacles the lower
 * end of which were visible are represented.
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(ObstaclesFieldPercept,
{
  /** The type of obstacle detected */
  ENUM(Type,
  {,
    unknown,            /**< No jersey detected. */
    ownPlayer,          /**< Jersey of a field player of the own team found. */
    opponentPlayer,     /**< Jersey of a field player of the opponent team found. */
    ownGoalkeeper,      /**< Jersey of the own team's goalkeeper found. */
    opponentGoalkeeper, /**< Jersey of the opponent team's goalkeeper found. */
  });

  /** Representation for a single obstacle */
  STREAMABLE(Obstacle,
  {,
    (Vector2f)(Vector2f::Zero()) center,     /**< Obstacle's center in robot-relative coordinates (in mm) */
    (Vector2f)(Vector2f::Zero()) left,       /**< Obstacle's left edge in robot-relative coordinates (in mm) */
    (Vector2f)(Vector2f::Zero()) right,      /**< Obstacle's right edge in robot-relative coordinates (in mm) */
    (bool)(false) fallen,                    /**< Is the obstacle a player lying on the field? */
    (Type)(Type::unknown) type,              /**< The type of the obstacle */
    (float)(0.f) confidence,                 /**< The type's confidence */
    (Matrix2f)(Matrix2f::Zero()) covariance, /**< The covariance of the center's measurement */
  });

  /** Draws this percept. */
  void draw() const;

  /** Checks whether all coordinates are finite. */
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< The obstacles with detected lower ends found in the current image. */
});
