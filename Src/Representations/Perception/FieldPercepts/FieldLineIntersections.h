/**
 * @file FieldLineIntersections.h
 * Declaration of a struct that represents the fieldline intersection percepts.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

/**
 * @struct FieldLineIntersections
 * A struct that represents the found intersections of fieldlines.
 */
STREAMABLE(FieldLineIntersections,
{
  /**
   * @struct Intersection
   * A struct representing an intersection of two fieldlines.
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
    });

    ENUM(AdditionalType,
    {,
      none,
      big, //< is added if the intersection belongs to long lines
      mid, //< is added if the intersection belongs to the midLine
    }),

    (IntersectionType) type,
    (AdditionalType)(AdditionalType::none) additionalType,
    (Vector2f) pos, /**< The fieldcoordinates of the intersection */
    /** dir1 and dir2 are the directions of the field lines.
     * If the type is T: dir1 shows the direction of the vertical line.
     *                   dir2 shows the direction of the horizontal line +90° relative to dir1.
     * dir1 and dir2 point always from the intersection towards the lines
     * If the type is L: dir2 is +90° relative to dir1*/
    (Vector2f) dir1,
    (Vector2f) dir2,

    (unsigned) ownIndex,
    (unsigned) indexDir1,
    (unsigned) indexDir2,
  });

  /**
   * The method draws the percepts to image/field/3D scene.
   */
  void draw() const,

  (std::vector<Intersection>) intersections, /**< The intersections of the lines */
});
