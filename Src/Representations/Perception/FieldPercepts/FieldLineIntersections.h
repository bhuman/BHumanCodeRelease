/**
 * @file FieldLineIntersections.h
 * Declaration of a struct that represents the field line intersection percepts.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Streaming/Enum.h"
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include <vector>

/**
 * @struct FieldLineIntersections
 * A struct that represents the found intersections of field lines.
 */
STREAMABLE(FieldLineIntersections,
{
  /**
   * @struct Intersection
   * A struct representing an intersection of two field lines.
   */
  STREAMABLE(Intersection,
  {
    /**
     * Intersection types are inclusive.
     * T includes L; X includes T and L */
    ENUM(IntersectionType,
    {,
      L,
      T,
      X,
    }),

    (IntersectionType) type,
    (Vector2f) pos, /**< The field coordinates of the intersection */
    (Matrix2f) cov, /**< The covariance of the intersection's measurement */
    /** dir1 and dir2 are the directions of the field lines.                             ___
     * If the type is T: dir1 shows the direction of the vertical line, i.e. the | in the |
     *                   dir2 shows the direction of the horizontal line (+90° relative to dir1).
     * dir1 and dir2 point always from the intersection away, along the lines
     * If the type is L: dir2 is +90° relative to dir1*/
    (Vector2f) dir1,
    (Vector2f) dir2,
  });

  /**
   * The method draws the percepts to image/field/3D scene.
   */
  void draw() const;

  /** Verifies that all intersections contain valid values */
  void verify() const,

  (std::vector<Intersection>) intersections, /**< The intersections of the lines */
});
