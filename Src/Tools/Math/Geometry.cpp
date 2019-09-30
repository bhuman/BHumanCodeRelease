/**
 * @file Math/Geometry.cpp
 * Implemets class Geometry
 *
 * @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
 * @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
 */

#include "Geometry.h"
#include "Approx.h"
#include "RotationMatrix.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>
#include <cstdlib>

using namespace std;

float Geometry::angleTo(const Pose2f& from, const Vector2f& to)
{
  const Pose2f relPos = Pose2f(to) - from;
  return atan2(relPos.translation.y(), relPos.translation.x());
}

void Geometry::Line::normalizeDirection()
{
  direction.normalize();
}

Geometry::Circle Geometry::getCircle(const Vector2i& point1, const Vector2i& point2, const Vector2i& point3)
{
  const float x1 = static_cast<float>(point1.x());
  const float y1 = static_cast<float>(point1.y());
  const float x2 = static_cast<float>(point2.x());
  const float y2 = static_cast<float>(point2.y());
  const float x3 = static_cast<float>(point3.x());
  const float y3 = static_cast<float>(point3.y());

  Circle circle;
  const float temp = x2 * y1 - x3 * y1 - x1 * y2 + x3 * y2 + x1 * y3 - x2 * y3;

  if(temp == 0)
    circle.radius = 0;
  else
    circle.radius = 0.5f *
                    sqrt(((sqr(x1 - x2) + sqr(y1 - y2)) *
                          (sqr(x1 - x3) + sqr(y1 - y3)) *
                          (sqr(x2 - x3) + sqr(y2 - y3))) /
                         sqr(temp));
  if(temp == 0)
    circle.center.x() = 0;
  else
    circle.center.x() = (sqr(x3) * (y1 - y2) +
                         (sqr(x1) + (y1 - y2) * (y1 - y3)) * (y2 - y3) +
                         sqr(x2) * (-y1 + y3)) /
                        (-2.0f * temp);
  if(temp == 0)
    circle.center.y() = 0;
  else
    circle.center.y() = (sqr(x1) * (x2 - x3) +
                         sqr(x2) * x3 +
                         x3 * (-sqr(y1) + sqr(y2)) -
                         x2 * (+sqr(x3) - sqr(y1) + sqr(y3)) +
                         x1 * (-sqr(x2) + sqr(x3) - sqr(y2) + sqr(y3))) /
                        (2.0f * temp);
  return circle;
}

void Geometry::PixeledLine::calculatePixels(const int x1, const int y1, const int x2, const int y2, const int stepSize)
{
  ASSERT(empty()); // only call from constructors
  if(x1 == x2 && y1 == y2)
    emplace_back(x1, y1);
  else
  {
    if(std::abs(x2 - x1) > std::abs(y2 - y1))
    {
      const int sign = sgn(x2 - x1);
      const int numberOfPixels = std::abs(x2 - x1) + 1;
      reserve(numberOfPixels / stepSize);
      for(int x = 0; x < numberOfPixels; x += stepSize)
      {
        const int y = x * (y2 - y1) / (x2 - x1);
        emplace_back(x1 + x * sign, y1 + y * sign);
      }
    }
    else
    {
      const int sign = sgn(y2 - y1);
      const int numberOfPixels = std::abs(y2 - y1) + 1;
      reserve(numberOfPixels / stepSize);
      for(int y = 0; y < numberOfPixels; y += stepSize)
      {
        const int x = y * (x2 - x1) / (y2 - y1);
        emplace_back(x1 + x * sign, y1 + y * sign);
      }
    }
  }
}

bool Geometry::getIntersectionOfLines(const Line& line1, const Line& line2, Vector2f& intersection)
{
  if(line1.direction.y() * line2.direction.x() == line1.direction.x() * line2.direction.y())
    return false;

  intersection.x() =
    line1.base.x() +
    line1.direction.x() *
    (line1.base.y() * line2.direction.x() -
     line2.base.y() * line2.direction.x() +
     (-line1.base.x() + line2.base.x()) * line2.direction.y()) /
    ((-line1.direction.y()) * line2.direction.x() + line1.direction.x() * line2.direction.y());

  intersection.y() =
    line1.base.y() +
    line1.direction.y() *
    (-line1.base.y() * line2.direction.x() +
     line2.base.y() * line2.direction.x() +
     (line1.base.x() - line2.base.x()) * line2.direction.y()) /
    (line1.direction.y() * line2.direction.x() - line1.direction.x() * line2.direction.y());

  return true;
}

int Geometry::getIntersectionOfCircles(const Circle& c0, const Circle& c1, Vector2f& p1, Vector2f& p2)
{
  // dx and dy are the vertical and horizontal distances between
  // the circle centers.
  const float dx = c1.center.x() - c0.center.x();
  const float dy = c1.center.y() - c0.center.y();

  // Determine the straight-line distance between the centers.
  const float d = sqrt((dy * dy) + (dx * dx));

  // Check for solvability.
  if(d > (c0.radius + c1.radius)) // no solution. circles do not intersect.
    return 0;
  if(d < abs(c0.radius - c1.radius)) //no solution. one circle is contained in the other
    return 0;

  // 'point 2' is the point where the line through the circle
  // intersection points crosses the line between the circle
  // centers.

  // Determine the distance from point 0 to point 2.
  const float a = ((c0.radius * c0.radius) - (c1.radius * c1.radius) + (d * d)) / (2.0f * d);

  // Determine the coordinates of point 2.
  const float x2 = c0.center.x() + (dx * a / d);
  const float y2 = c0.center.y() + (dy * a / d);

  // Determine the distance from point 2 to either of the
  // intersection points.
  const float h = sqrt((c0.radius * c0.radius) - (a * a));

  // Now determine the offsets of the intersection points from
  // point 2.
  const float rx = -dy * (h / d);
  const float ry = dx * (h / d);

  // Determine the absolute intersection points.
  p1.x() = x2 + rx;
  p2.x() = x2 - rx;
  p1.y() = y2 + ry;
  p2.y() = y2 - ry;

  return p1 == p2 ? 1 : 2;
}

int Geometry::getIntersectionOfLineAndCircle(const Line& line, const Circle& circle, Vector2f& firstIntersection, Vector2f& secondIntersection)
{
  /* solves the following system of equations:
   *
   * (x - x_m)^2 + (y - y_m)^2 = r^2
   * p + l * v = [x, y]
   *
   * where [x_m, y_m] is the center of the circle,
   * p is line.base and v is line.direction and
   * [x, y] is an intersection point.
   * Solution was found with the help of maple.
   */
  const float divisor = line.direction.squaredNorm();
  const float p = 2 * (line.base.dot(line.direction) - circle.center.dot(line.direction)) / divisor;
  const float q = ((line.base - circle.center).squaredNorm() - sqr(circle.radius)) / divisor;
  const float p_2 = p / 2.0f;
  const float radicand = sqr(p_2) - q;
  if(radicand < 0)
    return 0;
  else
  {
    const float radix = sqrt(radicand);
    firstIntersection = line.base + line.direction * (-p_2 + radix);
    secondIntersection = line.base + line.direction * (-p_2 - radix);
    return radicand == 0 ? 1 : 2;
  }
}

bool Geometry::getIntersectionOfRaysFactor(const Line& ray1, const Line& ray2, float& factor)
{
  const float divisor = ray2.direction.x() * ray1.direction.y() - ray1.direction.x() * ray2.direction.y();
  if(divisor == 0)
    return false;
  const float k = (ray2.direction.y() * ray1.base.x() - ray2.direction.y() * ray2.base.x() - ray2.direction.x() * ray1.base.y() + ray2.direction.x() * ray2.base.y()) / divisor;
  const float l = (ray1.direction.y() * ray1.base.x() - ray1.direction.y() * ray2.base.x() - ray1.direction.x() * ray1.base.y() + ray1.direction.x() * ray2.base.y()) / divisor;
  if((k >= 0) && (l >= 0) && (k <= 1) && (l <= 1))
  {
    factor = k;
    return true;
  }
  return false;
}

float Geometry::getDistanceToLine(const Line& line, const Vector2f& point)
{
  if(line.direction.x() == 0 && line.direction.y() == 0)
    return distance(point, line.base);

  Vector2f normal;
  normal.x() = line.direction.y();
  normal.y() = -line.direction.x();
  normal.normalize();

  const float c = normal.dot(line.base);

  return normal.dot(point) - c;
}

float Geometry::getDistanceToEdge(const Line& line, const Vector2f& point)
{
  if(line.direction.x() == 0 && line.direction.y() == 0)
    return distance(point, line.base);

  const float d = (point - line.base).dot(line.direction) / line.direction.dot(line.direction);

  if(d < 0)
    return distance(point, line.base);
  else if(d > 1.0f)
    return distance(point, line.base + line.direction);
  else
    return abs(getDistanceToLine(line, point));
}

float Geometry::distance(const Vector2f& point1, const Vector2f& point2)
{
  return (point2 - point1).norm();
}

float Geometry::distance(const Vector2i& point1, const Vector2i& point2)
{
  return (point2 - point1).cast<float>().norm();
}

bool Geometry::isPointInsideRectangle(const Vector2f& bottomLeftCorner, const Vector2f& topRightCorner, const Vector2f& point)
{
  return(bottomLeftCorner.x() <= point.x() && point.x() <= topRightCorner.x() &&
         bottomLeftCorner.y() <= point.y() && point.y() <= topRightCorner.y());
}

bool Geometry::isPointInsideRectangle2(const Vector2f& corner1, const Vector2f& corner2, const Vector2f& point)
{
  const Vector2f bottomLeft(std::min(corner1.x(), corner2.x()), std::min(corner1.y(), corner2.y()));
  const Vector2f topRight(std::max(corner1.x(), corner2.x()), std::max(corner1.y(), corner2.y()));
  return isPointInsideRectangle(bottomLeft, topRight, point);
}

bool Geometry::isPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, const Vector2i& point)
{
  return(bottomLeftCorner.x() <= point.x() && point.x() <= topRightCorner.x() &&
         bottomLeftCorner.y() <= point.y() && point.y() <= topRightCorner.y());
}

int ccw(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2)
{
  const float dx1 = p1.x() - p0.x();
  const float dy1 = p1.y() - p0.y();
  const float dx2 = p2.x() - p0.x();
  const float dy2 = p2.y() - p0.y();
  if(dx1 * dy2 > dy1 * dx2)
    return 1;
  if(dx1 * dy2 < dy1 * dx2)
    return -1;
  // Now (dx1*dy2 == dy1*dx2) must be true:
  if((dx1 * dx2 < 0.0f) || (dy1 * dy2 < 0.0f))
    return -1;
  if((dx1 * dx1 + dy1 * dy1) >= (dx2 * dx2 + dy2 * dy2))
    return 0;
  return 1;
}

bool Geometry::isPointInsideConvexPolygon(const Vector2f polygon[], const int numberOfPoints, const Vector2f& point)
{
  const int orientation = ccw(polygon[0], polygon[1], point);
  if(orientation == 0)
    return true;
  for(int i = 1; i < numberOfPoints; i++)
  {
    const int currentOrientation = ccw(polygon[i], polygon[(i + 1) % numberOfPoints], point);
    if(currentOrientation == 0)
      return true;
    if(currentOrientation != orientation)
      return false;
  }
  return true;
}

bool Geometry::isPointInsidePolygon(const Vector3f& point, const std::vector<Vector3f>& V)
{
  int i, j = static_cast<int>(V.size()) - 1;
  bool oddNodes = false;

  for(i = 0; i < static_cast<int>(V.size()); ++i)
  {
    if((V[i].y() < point.y() && V[j].y() >= point.y()) || (V[j].y() < point.y() && V[i].y() >= point.y()))
    {
      if(V[i].x() + (point.y() - V[i].y()) / (V[j].y() - V[i].y()) * (V[j].x() - V[i].x()) < point.x())
      {
        oddNodes = !oddNodes;
      }
    }
    j = i;
  }
  return oddNodes;
}

bool Geometry::checkIntersectionOfLines(const Vector2f& l1p1, const Vector2f& l1p2, const Vector2f& l2p1, const Vector2f& l2p2)
{
  return (((ccw(l1p1, l1p2, l2p1) * ccw(l1p1, l1p2, l2p2)) <= 0) &&
          ((ccw(l2p1, l2p2, l1p1) * ccw(l2p1, l2p2, l1p2)) <= 0));
}

bool Geometry::clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2i& point)
{
  bool clipped = false;
  if(point.x() < bottomLeftCorner.x())
  {
    point.x() = bottomLeftCorner.x();
    clipped = true;
  }
  if(point.x() > topRightCorner.x())
  {
    point.x() = topRightCorner.x();
    clipped = true;
  }
  if(point.y() < bottomLeftCorner.y())
  {
    point.y() = bottomLeftCorner.y();
    clipped = true;
  }
  if(point.y() > topRightCorner.y())
  {
    point.y() = topRightCorner.y();
    clipped = true;
  }
  return clipped;
}

bool Geometry::clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2f& point)
{
  bool clipped = false;
  if(point.x() < bottomLeftCorner.x())
  {
    point.x() = static_cast<float>(bottomLeftCorner.x());
    clipped = true;
  }
  if(point.x() > topRightCorner.x())
  {
    point.x() = static_cast<float>(topRightCorner.x());
    clipped = true;
  }
  if(point.y() < bottomLeftCorner.y())
  {
    point.y() = static_cast<float>(bottomLeftCorner.y());
    clipped = true;
  }
  if(point.y() > topRightCorner.y())
  {
    point.y() = static_cast<float>(topRightCorner.y());
    clipped = true;
  }
  return clipped;
}

bool Geometry::getIntersectionPointsOfLineAndRectangle(const Vector2i& bottomLeft, const Vector2i& topRight,
    const Geometry::Line& line, Vector2i& point1, Vector2i& point2)
{
  int foundPoints = 0;
  Vector2f point[2];
  if(line.direction.x() != 0)
  {
    const float y1 = line.base.y() + (bottomLeft.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if((y1 >= bottomLeft.y()) && (y1 <= topRight.y()))
    {
      point[foundPoints].x() = static_cast<float>(bottomLeft.x());
      point[foundPoints++].y() = y1;
    }
    const float y2 = line.base.y() + (topRight.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if((y2 >= bottomLeft.y()) && (y2 <= topRight.y()))
    {
      point[foundPoints].x() = static_cast<float>(topRight.x());
      point[foundPoints++].y() = y2;
    }
  }
  if(line.direction.y() != 0)
  {
    const float x1 = line.base.x() + (bottomLeft.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if((x1 >= bottomLeft.x()) && (x1 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x1;
      point[foundPoints].y() = static_cast<float>(bottomLeft.y());
      if((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
      {
        foundPoints++;
      }
    }
    const float x2 = line.base.x() + (topRight.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if((x2 >= bottomLeft.x()) && (x2 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x2;
      point[foundPoints].y() = static_cast<float>(topRight.y());
      if((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
      {
        foundPoints++;
      }
    }
  }
  switch(foundPoints)
  {
    case 1:
      point1 = point[0].cast<int>();
      point2 = point1;
      foundPoints++;
      return true;
    case 2:
      if((point[1] - point[0]).dot(line.direction) > 0)
      {
        point1 = point[0].cast<int>();
        point2 = point[1].cast<int>();
      }
      else
      {
        point1 = point[1].cast<int>();
        point2 = point[0].cast<int>();
      }
      return true;
    default:
      return false;
  }
}

bool Geometry::getIntersectionPointsOfLineAndRectangle(const Vector2f& bottomLeft, const Vector2f& topRight,
    const Geometry::Line& line, Vector2f& point1, Vector2f& point2)
{
  int foundPoints = 0;
  Vector2f point[2];
  if(line.direction.x() != 0)
  {
    const float y1 = line.base.y() + (bottomLeft.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if((y1 >= bottomLeft.y()) && (y1 <= topRight.y()))
    {
      point[foundPoints].x() = bottomLeft.x();
      point[foundPoints++].y() = y1;
    }
    const float y2 = line.base.y() + (topRight.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if((y2 >= bottomLeft.y()) && (y2 <= topRight.y()))
    {
      point[foundPoints].x() = topRight.x();
      point[foundPoints++].y() = y2;
    }
  }
  if(line.direction.y() != 0)
  {
    const float x1 = line.base.x() + (bottomLeft.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if((x1 >= bottomLeft.x()) && (x1 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x1;
      point[foundPoints].y() = bottomLeft.y();
      if((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
      {
        foundPoints++;
      }
    }
    const float x2 = line.base.x() + (topRight.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if((x2 >= bottomLeft.x()) && (x2 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x2;
      point[foundPoints].y() = topRight.y();
      if((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
        foundPoints++;
    }
  }
  switch(foundPoints)
  {
    case 1:
      point1 = point[0];
      point2 = point1;
      foundPoints++;
      return true;
    case 2:
      if((point[1] - point[0]).dot(line.direction) > 0)
      {
        point1 = point[0];
        point2 = point[1];
      }
      else
      {
        point1 = point[1];
        point2 = point[0];
      }
      return true;
    default:
      return false;
  }
}

bool Geometry::clipLineWithRectangleCohenSutherland(const Vector2i& topLeft, const Vector2i& bottomRight, Vector2i& point1, Vector2i& point2)
{
  constexpr int CLIPLEFT = 0b0001;
  constexpr int CLIPRIGHT = 0b0010;
  constexpr int CLIPLOWER = 0b0100;
  constexpr int CLIPUPPER = 0b1000;

  int K1 = 0, K2 = 0;

  const int dx = point2.x() - point1.x();
  const int dy = point2.y() - point1.y();

  if(point1.y() < topLeft.y())     K1 = CLIPLOWER;
  if(point1.y() > bottomRight.y()) K1 = CLIPUPPER;
  if(point1.x() < topLeft.x())     K1 |= CLIPLEFT;
  if(point1.x() > bottomRight.x()) K1 |= CLIPRIGHT;

  if(point2.y() < topLeft.y())     K2 = CLIPLOWER;
  if(point2.y() > bottomRight.y()) K2 = CLIPUPPER;
  if(point2.x() < topLeft.x())     K2 |= CLIPLEFT;
  if(point2.x() > bottomRight.x()) K2 |= CLIPRIGHT;

  while(K1 || K2)
  {
    if(K1 & K2)
      return false;

    if(K1)
    {
      if(K1 & CLIPLEFT)
      {
        point1.y() += (topLeft.x() - point1.x()) * dy / dx;
        point1.x() = topLeft.x();
      }
      else if(K1 & CLIPRIGHT)
      {
        point1.y() += (bottomRight.x() - point1.x()) * dy / dx;
        point1.x() = bottomRight.x();
      }
      if(K1 & CLIPLOWER)
      {
        point1.x() += (topLeft.y() - point1.y()) * dx / dy;
        point1.y() = topLeft.y();
      }
      else if(K1 & CLIPUPPER)
      {
        point1.x() += (bottomRight.y() - point1.y()) * dx / dy;
        point1.y() = bottomRight.y();
      }
      K1 = 0;

      if(point1.y() < topLeft.y())     K1 = CLIPLOWER;
      if(point1.y() > bottomRight.y()) K1 = CLIPUPPER;
      if(point1.x() < topLeft.x())     K1 |= CLIPLEFT;
      if(point1.x() > bottomRight.x()) K1 |= CLIPRIGHT;
    }

    if(K1 & K2)
      return false;

    if(K2)
    {
      if(K2 & CLIPLEFT)
      {
        point2.y() += (topLeft.x() - point2.x()) * dy / dx;
        point2.x() = topLeft.x();
      }
      else if(K2 & CLIPRIGHT)
      {
        point2.y() += (bottomRight.x() - point2.x()) * dy / dx;
        point2.x() = bottomRight.x();
      }
      if(K2 & CLIPLOWER)
      {
        point2.x() += (topLeft.y() - point2.y()) * dx / dy;
        point2.y() = topLeft.y();
      }
      else if(K2 & CLIPUPPER)
      {
        point2.x() += (bottomRight.y() - point2.y()) * dx / dy;
        point2.y() = bottomRight.y();
      }
      K2 = 0;

      if(point2.y() < topLeft.y())     K2 = CLIPLOWER;
      if(point2.y() > bottomRight.y()) K2 = CLIPUPPER;
      if(point2.x() < topLeft.x())     K2 |= CLIPLEFT;
      if(point2.x() > bottomRight.x()) K2 |= CLIPRIGHT;
    }
  }
  return true;
}

bool Geometry::isPointInsideTriangle(const float x1, const float y1, const float x2, const float y2,
                                     const float x3, const float y3, const float px, const float py)
{
  // calc  barycentric coordinates
  const float alpha = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) /
                      ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));
  const float beta = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) /
                     ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));
  const float gamma = 1.0f - alpha - beta;
  return alpha > 0 && beta > 0 && gamma > 0;
}

// from http://geomalgorithms.com/a07-_distance.html:
// Copyright 2001 softSurfer, 2012 Dan Sunday
// This code may be freely used, distributed and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.
float Geometry::distance(const LineSegment3D& S1, const LineSegment3D& S2, LineSegment3D& SE)
{
  const Vector3f u = S1.P1 - S1.P0;
  const Vector3f v = S2.P1 - S2.P0;
  const Vector3f w = S1.P0 - S2.P0;
  const float a = u.norm();         // always >= 0
  const float b = u.dot(v);
  const float c = v.norm();         // always >= 0
  const float d = u.dot(w);
  const float e = v.dot(w);
  const float D = a * c - b * b;    // always >= 0
  float sN, sD = D;       // sc = sN / sD, default sD = D >= 0
  float tN, tD = D;       // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if(Approx::isZero(D))   // the lines are almost parallel
  {
    sN = 0.0;         // force using point P0 on segment S1
    sD = 1.0;         // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  }
  else                   // get the closest points on the infinite lines
  {
    sN = (b * e - c * d);
    tN = (a * e - b * d);
    if(sN < 0.0)          // sc < 0 => the s=0 edge is visible
    {
      sN = 0.0;
      tN = e;
      tD = c;
    }
    else if(sN > sD)    // sc > 1  => the s=1 edge is visible
    {
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if(tN < 0.0)              // tc < 0 => the t=0 edge is visible
  {
    tN = 0.0;
    // recompute sc for this edge
    if(-d < 0.0)
      sN = 0.0;
    else if(-d > a)
      sN = sD;
    else
    {
      sN = -d;
      sD = a;
    }
  }
  else if(tN > tD)        // tc > 1  => the t=1 edge is visible
  {
    tN = tD;
    // recompute sc for this edge
    if((-d + b) < 0.0)
      sN = 0;
    else if((-d + b) > a)
      sN = sD;
    else
    {
      sN = (-d + b);
      sD = a;
    }
  }
  // finally do the division to get sc and tc
  const float sc = (Approx::isZero(sN) ? 0.f : sN / sD);
  const float tc = (Approx::isZero(tN) ? 0.f : tN / tD);

  // get the difference of the two closest points
  const Vector3f dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

  SE.P0 = S1.P0 + sc * u;
  SE.P1 = S2.P0 + tc * v;

  return dP.norm();   // return the closest distance
}

bool Geometry::isPointLeftOfLine(const Vector2f& start, const Vector2f& end, const Vector2f& point)
{
  return ((end.x() - start.x()) * (point.y() - start.y()) - (end.y() - start.y()) * (point.x() - start.x())) > 0.f;
}
