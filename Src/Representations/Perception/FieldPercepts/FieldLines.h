/**
 * @file FieldLines.h
 *
 * Declaration of a struct that represents perceived field lines in
 * field coordinates relative to the robot.
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct FieldLines
 * A struct that contains a list of perceived lines on the field
 */
STREAMABLE(FieldLines,
{
  /**
   * @struct Line
   * Internal representation for a single line
   */
  STREAMABLE(Line,
  {,
    (Angle) alpha,    /**< Direction of this line in Hesse norm form */
    (float) length,   /**< Distance between first and last point. Redundant but useful information */
    (Vector2f) first, /**< The starting point of this line in field coordinates relative to the robot */
    (Vector2f) last,  /**< The end point of this line in field coordinates relative to the robot */
  });

  /** The method draws the lines to image/field/3D scene. */
  void draw() const;

  /** Verifies that all lines contain valid values and are sorted by length. */
  void verify() const,

  (std::vector<Line>) lines, /**< The detected field lines, sorted by length (from long to short) */
});
