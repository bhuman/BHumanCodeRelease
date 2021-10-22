/**
 * @file Tools/Math/Geometry.h
 * Declares the namespace Geometry
 *
 * @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"

/**
 * The namespace Geometry contains representations for geometric objects and methods
 * for calculations with such objects.
 */
namespace Geometry
{
  /** Defines a circle by its center and its radius*/
  STREAMABLE(Circle,
  {
    Circle() = default;
    Circle(const Vector2f& c, float r),

    (Vector2f)(Vector2f::Zero()) center,
    (float)(0) radius,
  });

  inline Circle::Circle(const Vector2f& c, float r) :
    center(c), radius(r)
  {}

  /** Defines a rectangle by two points */
  STREAMABLE(Rect,
  {
    Rect() = default;
    Rect(const Vector2f& a, const Vector2f& b),

    (Vector2f)(Vector2f::Zero()) a,
    (Vector2f)(Vector2f::Zero()) b,
  });

  inline Rect::Rect(const Vector2f& a, const Vector2f& b) :
    a(a), b(b)
  {}

  /** Defines a line by two vectors*/
  STREAMABLE(Line,
  {
    Line() = default;
    Line(const Vector2f& base, const Vector2f& direction);
    Line(const Vector2i& base, const Vector2f& direction);
    Line(const Vector2i& base, const Vector2i& direction);
    Line(const Pose2f& base, float length = 1.f);
    Line(float baseX, float baseY, float directionX, float directionY);

    void normalizeDirection(),

    (Vector2f) base,
    (Vector2f) direction,
  });

  inline Line::Line(const Vector2f& base, const Vector2f& direction) :
    base(base), direction(direction)
  {}

  inline Line::Line(const Vector2i& base, const Vector2f& direction) :
    base(base.cast<float>()), direction(direction)
  {}

  inline Line::Line(const Vector2i& base, const Vector2i& direction) :
    base(base.cast<float>()), direction(direction.cast<float>())
  {}

  inline Line::Line(const Pose2f& base, float length) :
    base(base.translation), direction(Pose2f(base.rotation) * Vector2f(length, 0))
  {}

  inline Line::Line(float baseX, float baseY, float directionX, float directionY) :
    base(baseX, baseY), direction(directionX, directionY)
  {}

  struct PixeledLine : public std::vector<Vector2i>
  {
    PixeledLine(const int x1, const int y1, const int x2, const int y2, const int stepSize = 1)
    {
      calculatePixels(x1, y1, x2, y2, stepSize);
    }

    PixeledLine(const Vector2i& start, const Vector2i& end, const int stepSize = 1)
    {
      calculatePixels(start.x(), start.y(), end.x(), end.y(), stepSize);
    }

  private:
    void calculatePixels(const int x1, const int y1, const int x2, const int y2, const int stepSize);
  };

  /**
   * Calculates the angle between a pose and a position
   * @param from The base pose.
   * @param to The other position.
   * @return the angle from the pose to the position.
   */
  float angleTo(const Pose2f& from, const Vector2f& to);

  /**
   * Returns the circle defined by the three points.
   * @param point1 The first point.
   * @param point2 The second point.
   * @param point3 The third point.
   * @return The circle defined by point1, point2 and point3.
   */
  Circle getCircle(const Vector2i& point1, const Vector2i& point2, const Vector2i& point3);
  int getIntersectionOfCircles(const Circle& c1, const Circle& c2, Vector2f& p1, Vector2f& p2);

  /**
   * Computes the intersection point of a line and a circle.
   * @param line The Line.
   * @param circle The Circle.
   * @param firstIntersection The first intersection point, if there is one.
   * @param secondIntersection The second intersection point, if there is one.
   * @return The number of intersection points.
   */
  int getIntersectionOfLineAndCircle(const Line& line, const Circle& circle, Vector2f& firstIntersection, Vector2f& secondIntersection);
  bool checkIntersectionOfLines(const Vector2f& l1p1, const Vector2f& l1p2, const Vector2f& l2p1, const Vector2f& l2p2);
  [[nodiscard]] bool getIntersectionOfLines(const Line& line1, const Line& line2, Vector2f& intersection);
  [[nodiscard]] bool getIntersectionOfRaysFactor(const Line& ray1, const Line& ray2, float& intersection);
  [[nodiscard]] bool getIntersectionOfLineAndConvexPolygon(const std::vector<Vector2f>& polygon, const Line& direction, Vector2f& intersection);

  float getDistanceToLine(const Line& line, const Vector2f& point);
  float getDistanceToEdge(const Line& line, const Vector2f& point);

  float distance(const Vector2f& point1, const Vector2f& point2);
  float distance(const Vector2i& point1, const Vector2i& point2);

  [[nodiscard]] bool isPointInsideRectangle(const Vector2f& bottomLeftCorner, const Vector2f& topRightCorner, const Vector2f& point);
  [[nodiscard]] bool isPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, const Vector2i& point);
  [[nodiscard]] bool isPointInsideRectangle2(const Vector2f& corner1, const Vector2f& corner2, const Vector2f& point);
  [[nodiscard]] bool isPointInsideRectangle(const Rect& rect, const Vector2f& point);
  [[nodiscard]] bool isPointInsideConvexPolygon(const Vector2f polygon[], const int numberOfPoints, const Vector2f& point);
  [[nodiscard]] bool isPointInsidePolygon(const Vector3f& point, const std::vector<Vector3f>& V);
  [[nodiscard]] bool clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2i& point);
  [[nodiscard]] bool clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2f& point);

  /**
   * Clips a line with a rectangle
   * @param bottomLeft The bottom left corner of the rectangle
   * @param topRight The top right corner of the rectangle
   * @param line The line to be clipped
   * @param point1 The starting point of the resulting line
   * @param point2 The end point of the resulting line
   * @return states whether clipping was necessary (and done)
   */
  [[nodiscard]] bool getIntersectionPointsOfLineAndRectangle(const Vector2i& bottomLeft, const Vector2i& topRight,
                                                             const Geometry::Line& line, Vector2i& point1, Vector2i& point2);
  [[nodiscard]] bool getIntersectionPointsOfLineAndRectangle(const Vector2f& bottomLeft, const Vector2f& topRight,
                                                             const Geometry::Line& line, Vector2f& point1, Vector2f& point2);

  bool isPointLeftOfLine(const Vector2f& start, const Vector2f& end, const Vector2f& point);

  /**
   * Computes the projection of a point on a line
   * @param base The base point of the line
   * @param dir The direction vector of the line (has to be normalized to length 1)
   * @param point The point that is projected
   * @return A point on the line
   */
  Vector2f getOrthogonalProjectionOfPointOnLine(const Vector2f& base, const Vector2f& dir, const Vector2f& point);
};
