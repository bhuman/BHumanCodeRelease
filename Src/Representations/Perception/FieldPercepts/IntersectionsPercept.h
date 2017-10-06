#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

/**
 * @struct IntersectionsPercept
 * This struct contains all the intersections between lines fitted through linespots.
 */
STREAMABLE(IntersectionsPercept,
{
  STREAMABLE(Intersection,
  {
    ENUM(IntersectionType,
    {,
      L,
      T,
      X,
    });

    Intersection() = default;
    Intersection(const IntersectionType& t, const Vector2f& p, const Vector2f& d1, const Vector2f& d2, unsigned l1, unsigned l2)
    {
      type = t;
      pos = p;
      dir1 = d1;
      dir2 = d2;
      line1Index = l1;
      line2Index = l2;
    },

    (IntersectionType) type,
    (Vector2f) pos, /**< The fieldcoordinates of the intersection */
    (Vector2f) dir1, /**< The first direction of the lines intersected. (In field coordinates) */
    (Vector2f) dir2, /**< The second direction of the lines intersected. (In field coordinates) */
    (unsigned) line1Index, /**< The first line of the intersection*/
    (unsigned) line2Index, /**< The second line of the intersection*/
  });

  /**
   * The method draws all line spot intersections.
   */
  void draw() const,

  (std::vector<Intersection>) intersections,
});

inline void IntersectionsPercept::draw() const
{
  DEBUG_DRAWING("representation:IntersectionsPercept:field", "drawingOnField")
  {
    for(const Intersection& intersection : intersections)
    {
      CROSS("representation:IntersectionsPercept:field", intersection.pos.x(), intersection.pos.y(), 60, 30, Drawings::solidPen, ColorRGBA::blue);
    }
  }
}
