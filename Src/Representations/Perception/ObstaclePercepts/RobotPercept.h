/**
 * @file RobotPercept.h
 *
 * This file declares a representation that lists the robots that were detected in
 * the current image in robot-relative field coordinates.
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(RobotPercept,
{
  /** The type of robot detected */
  ENUM(Type,
  {,
    unknown,            /**< No jersey detected. */
    ownPlayer,          /**< Jersey of a field player of the own team found. */
    opponentPlayer,     /**< Jersey of a field player of the opponent team found. */
    ownGoalkeeper,      /**< Jersey of the own team's goalkeeper found. */
    opponentGoalkeeper, /**< Jersey of the opponent team's goalkeeper found. */
  });

  /** Representation for a single robot */
  STREAMABLE(Robot,
  {,
    (Vector2f)(Vector2f::Zero()) center,             /**< Robot's center in robot-relative coordinates (in mm) */
    (Vector2f)(Vector2f::Zero()) left,               /**< Robot's left edge in robot-relative coordinates (in mm) */
    (Vector2f)(Vector2f::Zero()) right,              /**< Robot's right edge in robot-relative coordinates (in mm) */
    (Vector2i)(Vector2i::Zero()) topLeftInImage,     /**< Top left corner of axis-aligned bounding box around robot in image */
    (Vector2i)(Vector2i::Zero()) bottomRightInImage, /**< Bottom right corner of axis-aligned bounding box around robot in image */
    (bool)(false) fallen,                            /**< Is the robot lying on the field? */
    (Type)(Type::unknown) type,                      /**< The type of the robot */
    (Matrix2f)(Matrix2f::Zero()) covariance,         /**< The covariance of the center's measurement */
  });

  /** Draws this percept. */
  void draw() const;

  /** Checks whether all coordinates are finite. */
  void verify() const,

  (std::vector<Robot>) robots, /**< The robots with detected lower ends found in the current image. */
});
