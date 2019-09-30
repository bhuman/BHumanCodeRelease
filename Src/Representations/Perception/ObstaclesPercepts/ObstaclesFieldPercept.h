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

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(ObstaclesFieldPercept,
{
  /** The type of obstacle detected. */
  ENUM(Type,
  {,
    unknown, /**< No jersey detected. */
    ownPlayer, /**< Jersey of own team found. */
    opponentPlayer, /**< Jersey of opponent team found. */
  });

  STREAMABLE(Obstacle,
  {,
    (Vector2f) center, /**< Obstacle's center in robot-relative coordinates (in mm). */
    (Vector2f) left, /**< Obstacle's left edge in robot-relative coordinates (in mm). */
    (Vector2f) right, /**< Obstacle's right edge in robot-relative coordinates (in mm). */
    (bool) fallen, /**< Is the obstacle a player lying on the field? */
    (Type) type, /**< The type of the obstacle. */
  });

  /** Draws this percept. */
  void draw() const;

  /** Checks whether all coordiantes are finite. */
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< The obstacles with detected lower ends found in the current image. */
});
