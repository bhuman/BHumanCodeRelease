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

  /** Representation for a single obstalce */
  STREAMABLE(Obstacle,
  {,
    (Vector2f) center,           /**< Obstacle's center in robot-relative coordinates (in mm) */
    (Vector2f) left,             /**< Obstacle's left edge in robot-relative coordinates (in mm) */
    (Vector2f) right,            /**< Obstacle's right edge in robot-relative coordinates (in mm) */
    (bool) fallen,               /**< Is the obstacle a player lying on the field? */
    (Type)(Type::unknown) type,  /**< The type of the obstacle */
    (float)(0.f) probability,    /**< The type's probability */
    (Matrix2f) covariance,       /**< The covariance of the center's measurement */
  });

  /** Draws this percept. */
  void draw() const;

  /** Checks whether all coordinates are finite. */
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< The obstacles with detected lower ends found in the current image. */
});
