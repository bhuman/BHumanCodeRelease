/**
 * @file LinePercept.h
 * Declaration of a struct that represents the fieldline percepts
 * @author jeff
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct LinePercept
 * A struct that represents the found fieldlines, center circle and intersections.
 */
STREAMABLE(LinePercept,
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

    (float) alpha, /**< direction of this line in Hesse norm form */
    (float) d, /**< distance of this line in Hesse norm form */
    (bool) midLine, /**< Whether this is the line throught the center circle */
    (Vector2f) first, /**< The starting point of this line in field coordinates */
    (Vector2f) last, /**< The end point of this line in field coordinates */
    (Vector2f) startInImage, /**< The start point of this line in image coordinates */
    (Vector2f) endInImage, /**< The end point of this line in image coordinates */
  });

  /**
   * @struct Circle
   * The center circle.
   */
  STREAMABLE(Circle,
  {,
    (Vector2f)(Vector2f::Zero()) pos, /**< The position of the center of the center circle in field coordinates */
    (bool)(false) found, /**< Whether the center circle was found in this frame */
    (unsigned)(0) lastSeen, /**< The last time the center circle was seen */
  });

  /**
   * @struct Intersection
   * A struct representing a intersection of two fieldlines
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
    (Vector2f) pos, /**< The fieldcoordinates of the intersection */
    /** dir1 and dir2 are the directions of the field lines.
     * If the type is T: dir1 shows the direction of the vertical line.
     *                   dir2 shows the direction of the horizontal line +90° relative to dir1.
     * dir1 and dir2 point always from the intersection towards the lines */
    (Vector2f) dir1,
    (Vector2f) dir2,
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
  (std::vector<Intersection>) intersections, /**< The intersections of the lines */
  (Circle) circle, /**< The position of the center circle if found */
});
