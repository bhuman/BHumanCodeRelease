/**
 * @file FieldLines.h
 * Declaration of a struct that represents the fieldline percepts
 * @author jeff
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct FieldLines
 * A struct that represents the found fieldlines.
 */
STREAMABLE(FieldLines,
{
  /**
   * @struct Line
   *
   * This struct represents a found fieldline.
   */
  STREAMABLE(Line,
  {
    /**
     * Calculates the distance of a point p to this line
     * @param p a point
     * @return the distance
     */
    float calculateDistToLine(const Vector2f& p) const
    {
      return p.x() * std::cos(alpha) + p.y() * std::sin(alpha) - d;
    }

    /**
     * Calculates the closest point to a point on this line
     * @param p a point
     * @return the closest point on this line
     */
    Vector2f calculateClosestPointOnLine(const Vector2f& p) const,

    (Angle) alpha, /**< direction of this line in Hesse norm form */
    (float) d, /**< distance of this line in Hesse norm form */
    (bool) midLine, /**< Whether this is the line throught the center circle */
    (bool) isLong, /**< Whether this is a long line */
    (Vector2f) first, /**< The starting point of this line in field coordinates */
    (Vector2f) last, /**< The end point of this line in field coordinates */
  });

  /** Determines the closest line to a given point
   * @param point the given point
   * @param retLine the closest line
   * @return the distance from point to retLine
   * */
  float getClosestLine(Vector2f point, Line& retLine) const;

  /**
   * The method draws the percepts to image/field/3D scene.
   */
  void draw() const,

  (std::vector<Line>) lines, /**< The found fieldlines */
});
