/**
* @file Math/Geometry.cpp
* Implemets class Geometry
*
* @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
* @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
*/

#include "Geometry.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include <algorithm>

using namespace std;

const double MAX_DIST_ON_FIELD = sqrt(10400.f*10400.f + 7400.f*7400.f);


float Geometry::angleTo(const Pose2D& from, const Vector2<>& to)
{
  Pose2D relPos = Pose2D(to) - from;
  return atan2(relPos.translation.y, relPos.translation.x);
}

float Geometry::distanceTo(const Pose2D& from, const Vector2<>& to)
{
  return (Pose2D(to) - from).translation.abs();
}

Vector2<> Geometry::vectorTo(const Pose2D& from, const Vector2<>& to)
{
  return (Pose2D(to) - from).translation;
}

void Geometry::Line::normalizeDirection()
{
  float distance = sqrt(sqr(direction.x) + sqr(direction.y));
  direction.x = direction.x / distance;
  direction.y = direction.y / distance;
}

Geometry::Circle Geometry::getCircle
(
  const Vector2<int>& point1,
  const Vector2<int>& point2,
  const Vector2<int>& point3
)
{
  float x1 = (float)point1.x;
  float y1 = (float)point1.y;
  float x2 = (float)point2.x;
  float y2 = (float)point2.y;
  float x3 = (float)point3.x;
  float y3 = (float)point3.y;

  Circle circle;

  const float temp = x2 * y1 - x3 * y1 - x1 * y2 + x3 * y2 + x1 * y3 - x2 * y3;

  if(temp == 0)
  {
    circle.radius = 0;
  }
  else
  {
    circle.radius =
      0.5f *
      sqrt(
        ((sqr(x1 - x2) + sqr(y1 - y2)) *
         (sqr(x1 - x3) + sqr(y1 - y3)) *
         (sqr(x2 - x3) + sqr(y2 - y3))
        )
        /
        sqr(temp)
      );
  }

  if(temp == 0)
  {
    circle.center.x = 0;
  }
  else
  {
    circle.center.x =
      (
        sqr(x3) * (y1 - y2) +
        (sqr(x1) + (y1 - y2) * (y1 - y3)) * (y2 - y3) +
        sqr(x2) * (-y1 + y3)
      )
      /
      (-2.0f * temp);
  }

  if(temp == 0)
  {
    circle.center.y = 0;
  }
  else
  {
    circle.center.y =
      (
        sqr(x1) * (x2 - x3) +
        sqr(x2) * x3 +
        x3 * (-sqr(y1) + sqr(y2)) -
        x2 * (+sqr(x3) - sqr(y1) + sqr(y3)) +
        x1 * (-sqr(x2) + sqr(x3) - sqr(y2) + sqr(y3))
      )
      /
      (2.0f * temp);
  }
  return circle;
}

bool Geometry::getIntersectionOfLines
(
  const Line& line1,
  const Line& line2,
  Vector2<int>& intersection
)
{
  Vector2<> intersectionDouble;
  bool toReturn = getIntersectionOfLines(line1, line2, intersectionDouble);
  intersection.x = (int)intersectionDouble.x;
  intersection.y = (int)intersectionDouble.y;
  return toReturn;
}

bool Geometry::getIntersectionOfLines
(
  const Line& line1,
  const Line& line2,
  Vector2<>& intersection
)
{
  if(line1.direction.y* line2.direction.x == line1.direction.x * line2.direction.y)
  {
    return false;
  }

  intersection.x =
    line1.base.x +
    line1.direction.x *
    (
      line1.base.y * line2.direction.x -
      line2.base.y * line2.direction.x +
      (-line1.base.x + line2.base.x) * line2.direction.y
    )
    /
    ((-line1.direction.y) * line2.direction.x + line1.direction.x * line2.direction.y);

  intersection.y =
    line1.base.y +
    line1.direction.y *
    (
      -line1.base.y * line2.direction.x +
      line2.base.y * line2.direction.x +
      (line1.base.x - line2.base.x) * line2.direction.y
    )
    /
    (line1.direction.y * line2.direction.x - line1.direction.x * line2.direction.y);

  return true;
}


int Geometry::getIntersectionOfCircles(
  const Circle& c0,
  const Circle& c1,
  Vector2<> &p1,
  Vector2<> &p2
)
{
  float a, dx, dy, d, h, rx, ry;
  float x2, y2;

  /* dx and dy are the vertical and horizontal distances between
   * the circle centers.
   */
  dx = c1.center.x - c0.center.x;
  dy = c1.center.y - c0.center.y;

  /* Determine the straight-line distance between the centers. */
  d = sqrt((dy * dy) + (dx * dx));

  /* Check for solvability. */
  if(d > (c0.radius + c1.radius))
  {
    /* no solution. circles do not intersect. */
    return 0;
  }
  if(d < abs(c0.radius - c1.radius))
  {
    /* no solution. one circle is contained in the other */
    return 0;
  }

  /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle
   * centers.
   */

  /* Determine the distance from point 0 to point 2. */
  a = ((c0.radius * c0.radius) - (c1.radius * c1.radius) + (d * d)) / (2.0f * d) ;

  /* Determine the coordinates of point 2. */
  x2 = c0.center.x + (dx * a / d);
  y2 = c0.center.y + (dy * a / d);

  /* Determine the distance from point 2 to either of the
   * intersection points.
   */
  h = sqrt((c0.radius * c0.radius) - (a * a));

  /* Now determine the offsets of the intersection points from
   * point 2.
   */
  rx = -dy * (h / d);
  ry =  dx * (h / d);

  /* Determine the absolute intersection points. */
  p1.x = x2 + rx;
  p2.x = x2 - rx;
  p1.y = y2 + ry;
  p2.y = y2 - ry;

  return 1;
}

int Geometry::getIntersectionOfLineAndCircle(
  const Line& line,
  const Circle& circle,
  Vector2<>& firstIntersection,
  Vector2<>& secondIntersection
)
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
  const float divisor = line.direction.squareAbs();
  const float p = 2 * (line.base * line.direction - circle.center * line.direction) / divisor;
  const float q = ((line.base - circle.center).sqr() - sqr(circle.radius)) / divisor;
  const float p_2 = p / 2.0f;
  const float radicand = sqr(p_2) - q;
  if(radicand < 0)
  {
    return 0;
  }
  else
  {
    const float radix = sqrt(radicand);
    firstIntersection = line.base + line.direction * (-p_2 + radix);
    secondIntersection = line.base + line.direction * (-p_2 - radix);
    return radicand == 0 ? 1 : 2;
  }
}

bool Geometry::getIntersectionOfRaysFactor
(
  const Line& ray1,
  const Line& ray2,
  float& factor
)
{
  float divisor = ray2.direction.x * ray1.direction.y - ray1.direction.x * ray2.direction.y;
  if(divisor == 0)
  {
    return false;
  }
  float k = (ray2.direction.y * ray1.base.x - ray2.direction.y * ray2.base.x - ray2.direction.x * ray1.base.y + ray2.direction.x * ray2.base.y) / divisor;
  float l = (ray1.direction.y * ray1.base.x - ray1.direction.y * ray2.base.x - ray1.direction.x * ray1.base.y + ray1.direction.x * ray2.base.y) / divisor;
  if((k >= 0) && (l >= 0) && (k <= 1) && (l <= 1))
  {
    factor = k;
    return true;
  }
  return false;
}

float Geometry::getDistanceToLine
(
  const Line& line,
  const Vector2<>& point
)
{
  if(line.direction.x == 0 && line.direction.y == 0)
    return distance(point, line.base);

  Vector2<> normal;
  normal.x = line.direction.y;
  normal.y = -line.direction.x;
  normal.normalize();

  float c = normal * line.base;

  return normal * point - c;
}

float Geometry::getDistanceToEdge
(
  const Line& line,
  const Vector2<>& point
)
{
  if(line.direction.x == 0 && line.direction.y == 0)
    return distance(point, line.base);

  float c = line.direction * line.base;

  float d = (line.direction * point - c) / (line.direction * line.direction);

  if(d < 0)
    return distance(point, line.base);
  else if(d > 1.0f)
    return distance(point, line.base + line.direction);
  else
    return abs(getDistanceToLine(line, point));
}


float Geometry::distance
(
  const Vector2<>& point1,
  const Vector2<>& point2
)
{
  return (point2 - point1).abs();
}

float Geometry::distance
(
  const Vector2<int>& point1,
  const Vector2<int>& point2
)
{
  return (float)(point2 - point1).abs();
}

void Geometry::calculateAnglesForPoint
(
  const Vector2<>& point,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  Vector2<>& angles
)
{
  float factor = cameraInfo.focalLength;

  Vector3<> vectorToPoint(
    factor,
    cameraInfo.opticalCenter.x - point.x,
    cameraInfo.opticalCenter.y - point.y);

  Vector3<> vectorToPointWorld =
    cameraMatrix.rotation * vectorToPoint;

  angles.x =
    atan2(vectorToPointWorld.y, vectorToPointWorld.x);

  angles.y =
    atan2(
      vectorToPointWorld.z,
      sqrt(sqr(vectorToPointWorld.x) + sqr(vectorToPointWorld.y))
    );
}

bool Geometry::calculatePointByAngles
(
  const Vector2<>& angles,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  Vector2<int>& point
)
{
  Vector3<> vectorToPointWorld, vectorToPoint;
  vectorToPointWorld.x = cos(angles.x);
  vectorToPointWorld.y = sin(angles.x);
  vectorToPointWorld.z = tan(angles.y);

  RotationMatrix rotationMatrix = cameraMatrix.rotation;
  vectorToPoint = rotationMatrix.invert() * vectorToPointWorld;

  float factor = cameraInfo.focalLength;

  float scale = factor / vectorToPoint.x;

  point.x = (int)(0.5f + cameraInfo.opticalCenter.x - vectorToPoint.y * scale);
  point.y = (int)(0.5f + cameraInfo.opticalCenter.y  - vectorToPoint.z * scale);
  return vectorToPoint.x > 0;
}

bool Geometry::clipLineWithQuadrangle
(
  const Line& lineToClip,
  const Vector2<>& corner0,
  const Vector2<>& corner1,
  const Vector2<>& corner2,
  const Vector2<>& corner3,
  Vector2<int>& clipPoint1,
  Vector2<int>& clipPoint2
)
{
  Vector2<> point1, point2;
  bool toReturn = clipLineWithQuadrangle(
                    lineToClip,
                    corner0, corner1, corner2, corner3,
                    point1, point2);
  clipPoint1.x = (int)point1.x;
  clipPoint1.y = (int)point1.y;
  clipPoint2.x = (int)point2.x;
  clipPoint2.y = (int)point2.y;
  return toReturn;
}

bool Geometry::clipLineWithQuadrangle
(
  const Line& lineToClip,
  const Vector2<>& corner0,
  const Vector2<>& corner1,
  const Vector2<>& corner2,
  const Vector2<>& corner3,
  Vector2<>& clipPoint1,
  Vector2<>& clipPoint2
)
{
  Geometry::Line side[4] , verticalLine;

  verticalLine.base = lineToClip.base;

  verticalLine.direction.x = -lineToClip.direction.y;
  verticalLine.direction.y = lineToClip.direction.x;

  Vector2<> corner[4];
  corner[0] = corner0;
  corner[1] = corner1;
  corner[2] = corner2;
  corner[3] = corner3;

  side[0].base = corner0;
  side[0].direction = corner1;

  side[1].base = corner1;
  side[1].direction = corner3;

  side[2].base = corner2;
  side[2].direction = corner1;

  side[3].base = corner3;
  side[3].direction = corner3;

  Vector2<> point1, point2, point;
  bool nextIsPoint1 = true;

  if(Geometry::getIntersectionOfLines(side[0], lineToClip, point))
  {
    if(corner[0].x < point.x && point.x < corner[1].x)
    {
      if(nextIsPoint1)
      {
        point1 = point;
        nextIsPoint1 = false;
      }
    }
  }

  if(Geometry::getIntersectionOfLines(side[1], lineToClip, point))
  {
    if(corner[1].y < point.y && point.y < corner[2].y)
    {
      if(nextIsPoint1)
      {
        point1 = point;
        nextIsPoint1 = false;
      }
      else
        point2 = point;
    }
  }

  if(Geometry::getIntersectionOfLines(side[2], lineToClip, point))
  {
    if(corner[2].x > point.x && point.x > corner[3].x)
    {
      if(nextIsPoint1)
      {
        point1 = point;
        nextIsPoint1 = false;
      }
      else
        point2 = point;
    }
  }

  if(Geometry::getIntersectionOfLines(side[3], lineToClip, point))
  {
    if(corner[3].y > point.y && point.y > corner[0].y)
    {
      if(nextIsPoint1)
      {
        point1 = point;
        nextIsPoint1 = false;
      }
      else
        point2 = point;
    }
  }

  if(nextIsPoint1)
    return false;

  if(getDistanceToLine(verticalLine, point1) < getDistanceToLine(verticalLine, point2))
  {
    clipPoint1 = point1;
    clipPoint2 = point2;
  }
  else
  {
    clipPoint1 = point2;
    clipPoint2 = point1;
  }
  return true;
}

bool Geometry::isPointInsideRectangle
(
  const Vector2<>& bottomLeftCorner,
  const Vector2<>& topRightCorner,
  const Vector2<>& point
)
{
  return(
          bottomLeftCorner.x <= point.x && point.x <= topRightCorner.x &&
          bottomLeftCorner.y <= point.y && point.y <= topRightCorner.y
        );
}

bool Geometry::isPointInsideRectangle2
(
  const Vector2<>& corner1,
  const Vector2<>& corner2,
  const Vector2<>& point
)
{
  Vector2<> bottomLeft(std::min(corner1.x, corner2.x), std::min(corner1.y, corner2.y));
  Vector2<> topRight(std::max(corner1.x, corner2.x), std::max(corner1.y, corner2.y));
  return isPointInsideRectangle(bottomLeft, topRight, point);
}


bool Geometry::isPointInsideRectangle
(
  const Vector2<int>& bottomLeftCorner,
  const Vector2<int>& topRightCorner,
  const Vector2<int>& point
)
{
  return(
          bottomLeftCorner.x <= point.x && point.x <= topRightCorner.x &&
          bottomLeftCorner.y <= point.y && point.y <= topRightCorner.y
        );
}

int Geometry::ccw(const Vector2<>& p0, const Vector2<>& p1, const Vector2<>& p2)
{
  float dx1(p1.x - p0.x);
  float dy1(p1.y - p0.y);
  float dx2(p2.x - p0.x);
  float dy2(p2.y - p0.y);
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

bool Geometry::isPointInsideConvexPolygon(
  const Vector2<> polygon[],
  const int numberOfPoints,
  const Vector2<>& point)
{
  int orientation(ccw(polygon[0], polygon[1], point));
  if(orientation == 0)
    return true;
  for(int i = 1; i < numberOfPoints; i++)
  {
    int currentOrientation(ccw(polygon[i], polygon[(i + 1) % numberOfPoints], point));
    if(currentOrientation == 0)
      return true;
    if(currentOrientation != orientation)
      return false;
  }
  return true;
}

bool Geometry::checkIntersectionOfLines(
  const Vector2<>& l1p1,
  const Vector2<>& l1p2,
  const Vector2<>& l2p1,
  const Vector2<>& l2p2)
{
  return (((ccw(l1p1, l1p2, l2p1) * ccw(l1p1, l1p2, l2p2)) <= 0)
          && ((ccw(l2p1, l2p2, l1p1) * ccw(l2p1, l2p2, l1p2)) <= 0));
}

bool Geometry::clipPointInsideRectangle(
  const Vector2<int>& bottomLeftCorner,
  const Vector2<int>& topRightCorner,
  Vector2<int>& point
)
{
  bool clipped = false;
  if(point.x < bottomLeftCorner.x)
  {
    point.x = bottomLeftCorner.x;
    clipped = true;
  }
  if(point.x > topRightCorner.x)
  {
    point.x = topRightCorner.x;
    clipped = true;
  }
  if(point.y < bottomLeftCorner.y)
  {
    point.y = bottomLeftCorner.y;
    clipped = true;
  }
  if(point.y > topRightCorner.y)
  {
    point.y = topRightCorner.y;
    clipped = true;
  }
  return clipped;
}

bool Geometry::clipPointInsideRectangle(
  const Vector2<int>& bottomLeftCorner,
  const Vector2<int>& topRightCorner,
  Vector2<>& point
)
{
  bool clipped = false;
  if(point.x < bottomLeftCorner.x)
  {
    point.x = (float)bottomLeftCorner.x;
    clipped = true;
  }
  if(point.x > topRightCorner.x)
  {
    point.x = (float)topRightCorner.x;
    clipped = true;
  }
  if(point.y < bottomLeftCorner.y)
  {
    point.y = (float)bottomLeftCorner.y;
    clipped = true;
  }
  if(point.y > topRightCorner.y)
  {
    point.y = (float)topRightCorner.y;
    clipped = true;
  }
  return clipped;
}

bool Geometry::calculatePointOnFieldHacked
(
  const int x,
  const int y,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  Vector2<>& pointOnField
)
{
  float xFactor = cameraInfo.focalLengthInv,
        yFactor = cameraInfo.focalLengthInv;

  Vector3<> vectorToCenter(1, float(cameraInfo.opticalCenter.x - x) * xFactor,
                           float(cameraInfo.opticalCenter.y - y) * yFactor);

  Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;

  //Is the point above the horizon ? - return
  if(vectorToCenterWorld.z > -5 * yFactor) return false;

  float a1 = cameraMatrix.translation.x,
        a2 = cameraMatrix.translation.y,
        a3 = cameraMatrix.translation.z,
        b1 = vectorToCenterWorld.x,
        b2 = vectorToCenterWorld.y,
        b3 = vectorToCenterWorld.z,
        f = a3 / b3;

  pointOnField.x = a1 - f * b1;
  pointOnField.y = a2 - f * b2;

  return true;
}

bool Geometry::calculatePointOnField
(
  const int x,
  const int y,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  Vector2<>& pointOnField
)
{
  float xFactor = cameraInfo.focalLengthInv,
        yFactor = cameraInfo.focalLengthInv;

  Vector3<> vectorToCenter(1, float(cameraInfo.opticalCenter.x - x) * xFactor,
                           float(cameraInfo.opticalCenter.y - y) * yFactor);

  Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;

  //Is the point above the horizon ? - return
  if(vectorToCenterWorld.z > -5 * yFactor) return false;

  float a1 = cameraMatrix.translation.x,
        a2 = cameraMatrix.translation.y,
        a3 = cameraMatrix.translation.z ,
        b1 = vectorToCenterWorld.x,
        b2 = vectorToCenterWorld.y,
        b3 = vectorToCenterWorld.z,
        f = a3 / b3;

  pointOnField.x = a1 - f * b1;
  pointOnField.y = a2 - f * b2;

  return abs(pointOnField.x) < MAX_DIST_ON_FIELD && abs(pointOnField.y) < MAX_DIST_ON_FIELD;
}

bool Geometry::calculatePointOnField(const Vector2<>& point, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2<>& pointOnField)
{
  Vector3<> vectorToCenter(1, float(cameraInfo.opticalCenter.x - point.x) * cameraInfo.focalLengthInv, float(cameraInfo.opticalCenter.y - point.y) * cameraInfo.focalLengthInv);
  Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;

  //Is the point above the horizon ? - return
  if(vectorToCenterWorld.z > -5 * cameraInfo.focalLengthInv) return false;

  float a1 = cameraMatrix.translation.x,
        a2 = cameraMatrix.translation.y,
        a3 = cameraMatrix.translation.z ,
        b1 = vectorToCenterWorld.x,
        b2 = vectorToCenterWorld.y,
        b3 = vectorToCenterWorld.z,
        f = a3 / b3;

  pointOnField.x = a1 - f * b1;
  pointOnField.y = a2 - f * b2;

  return abs(pointOnField.x) < MAX_DIST_ON_FIELD && abs(pointOnField.y) < MAX_DIST_ON_FIELD;
}

bool Geometry::calculatePointOnField(const Vector2<>& image, const float& fieldZ, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector3<>& field)
{
  Vector3<> unscaledCamera(cameraInfo.focalLength, float(cameraInfo.opticalCenter.x - image.x), float(cameraInfo.opticalCenter.y - image.y));
  const Vector3<> unscaledField(cameraMatrix.rotation * unscaledCamera);

  if(fieldZ < cameraMatrix.translation.z)
  {
    if(unscaledField.z > 0) return false;
  }
  else
  {
    if(unscaledField.z < 0) return false;
  }

  const float scale((cameraMatrix.translation.z - (float) fieldZ) / unscaledField.z);
  field.x = cameraMatrix.translation.x - scale * unscaledField.x;
  field.y = cameraMatrix.translation.y - scale * unscaledField.y;
  field.z = (float) fieldZ;
  return true;
}


bool Geometry::calculatePointInImage
(
  const Vector2<>& point,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  Vector2<int>& pointInImage
)
{
  Vector2<> offset(point.x - cameraMatrix.translation.x,
                   point.y - cameraMatrix.translation.y);
  return calculatePointByAngles(
           Vector2<>(atan2(offset.y, offset.x),
                     -atan2(cameraMatrix.translation.z, offset.abs())),
           cameraMatrix, cameraInfo,
           pointInImage
         );
}

bool Geometry::calculatePointInImage
(
  const Vector3<>& point,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  Vector2<>& pointInImage
)
{
  Vector3<> pointInCam = cameraMatrix.invert() * point;
  if(pointInCam.x == 0)
  {
    return false;
  }
  pointInCam *= cameraInfo.focalLength / pointInCam.x;
  pointInImage = cameraInfo.opticalCenter - Vector2<>(pointInCam.y, pointInCam.z);
  return pointInCam.x > 0;
}


bool Geometry::getIntersectionPointsOfLineAndRectangle(
  const Vector2<int>& bottomLeft,
  const Vector2<int>& topRight,
  const Geometry::Line line,
  Vector2<int>& point1,
  Vector2<int>& point2
)
{
  int foundPoints = 0;
  Vector2<> point[2];
  if(line.direction.x != 0)
  {
    float y1 = line.base.y + (bottomLeft.x - line.base.x) * line.direction.y / line.direction.x;
    if((y1 >= bottomLeft.y) && (y1 <= topRight.y))
    {
      point[foundPoints].x = (float) bottomLeft.x;
      point[foundPoints++].y = y1;
    }
    float y2 = line.base.y + (topRight.x - line.base.x) * line.direction.y / line.direction.x;
    if((y2 >= bottomLeft.y) && (y2 <= topRight.y))
    {
      point[foundPoints].x = (float) topRight.x;
      point[foundPoints++].y = y2;
    }
  }
  if(line.direction.y != 0)
  {
    float x1 = line.base.x + (bottomLeft.y - line.base.y) * line.direction.x / line.direction.y;
    if((x1 >= bottomLeft.x) && (x1 <= topRight.x) && (foundPoints < 2))
    {
      point[foundPoints].x = x1;
      point[foundPoints].y = (float) bottomLeft.y;
      if((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1))
      {
        foundPoints++;
      }
    }
    float x2 = line.base.x + (topRight.y - line.base.y) * line.direction.x / line.direction.y;
    if((x2 >= bottomLeft.x) && (x2 <= topRight.x) && (foundPoints < 2))
    {
      point[foundPoints].x = x2;
      point[foundPoints].y = (float) topRight.y;
      if((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1))
      {
        foundPoints++;
      }
    }
  }
  switch(foundPoints)
  {
  case 1:
    point1.x = (int)point[0].x;
    point2.x = point1.x;
    point1.y = (int)point[0].y;
    point2.y = point1.y;
    foundPoints++;
    return true;
  case 2:
    if((point[1] - point[0])*line.direction > 0)
    {
      point1.x = (int)point[0].x;
      point1.y = (int)point[0].y;
      point2.x = (int)point[1].x;
      point2.y = (int)point[1].y;
    }
    else
    {
      point1.x = (int)point[1].x;
      point1.y = (int)point[1].y;
      point2.x = (int)point[0].x;
      point2.y = (int)point[0].y;
    }
    return true;
  default:
    return false;
  }
}

bool Geometry::getIntersectionPointsOfLineAndRectangle(
  const Vector2<>& bottomLeft,
  const Vector2<>& topRight,
  const Geometry::Line line,
  Vector2<>& point1,
  Vector2<>& point2
)
{
  int foundPoints = 0;
  Vector2<> point[2];
  if(line.direction.x != 0)
  {
    float y1 = line.base.y + (bottomLeft.x - line.base.x) * line.direction.y / line.direction.x;
    if((y1 >= bottomLeft.y) && (y1 <= topRight.y))
    {
      point[foundPoints].x = bottomLeft.x;
      point[foundPoints++].y = y1;
    }
    float y2 = line.base.y + (topRight.x - line.base.x) * line.direction.y / line.direction.x;
    if((y2 >= bottomLeft.y) && (y2 <= topRight.y))
    {
      point[foundPoints].x = topRight.x;
      point[foundPoints++].y = y2;
    }
  }
  if(line.direction.y != 0)
  {
    float x1 = line.base.x + (bottomLeft.y - line.base.y) * line.direction.x / line.direction.y;
    if((x1 >= bottomLeft.x) && (x1 <= topRight.x) && (foundPoints < 2))
    {
      point[foundPoints].x = x1;
      point[foundPoints].y = bottomLeft.y;
      if((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1))
      {
        foundPoints++;
      }
    }
    float x2 = line.base.x + (topRight.y - line.base.y) * line.direction.x / line.direction.y;
    if((x2 >= bottomLeft.x) && (x2 <= topRight.x) && (foundPoints < 2))
    {
      point[foundPoints].x = x2;
      point[foundPoints].y = topRight.y;
      if((foundPoints == 0) || ((point[0] - point[1]).abs() > 0.1))
      {
        foundPoints++;
      }
    }
  }
  switch(foundPoints)
  {
  case 1:
    point1.x = point[0].x;
    point2.x = point1.x;
    point1.y = point[0].y;
    point2.y = point1.y;
    foundPoints++;
    return true;
  case 2:
    if((point[1] - point[0])*line.direction > 0)
    {
      point1.x = point[0].x;
      point1.y = point[0].y;
      point2.x = point[1].x;
      point2.y = point[1].y;
    }
    else
    {
      point1.x = point[1].x;
      point1.y = point[1].y;
      point2.x = point[0].x;
      point2.y = point[0].y;
    }
    return true;
  default:
    return false;
  }
}
#define CLIPLEFT  1  // 0001
#define CLIPRIGHT 2  // 0010
#define CLIPLOWER 4  // 0100
#define CLIPUPPER 8  // 1000

bool Geometry::clipLineWithRectangleCohenSutherland
(
  const Vector2<int>& topLeft,
  const Vector2<int>& bottomRight,
  Vector2<int>& point1,
  Vector2<int>& point2
)
{
  int K1 = 0, K2 = 0;
  int dx, dy;

  dx = point2.x - point1.x;
  dy = point2.y - point1.y;

  if(point1.y < topLeft.y)     K1  = CLIPLOWER;
  if(point1.y > bottomRight.y) K1  = CLIPUPPER;
  if(point1.x < topLeft.x)     K1 |= CLIPLEFT;
  if(point1.x > bottomRight.x) K1 |= CLIPRIGHT;

  if(point2.y < topLeft.y)     K2  = CLIPLOWER;
  if(point2.y > bottomRight.y) K2  = CLIPUPPER;
  if(point2.x < topLeft.x)     K2 |= CLIPLEFT;
  if(point2.x > bottomRight.x) K2 |= CLIPRIGHT;

  while(K1 || K2)
  {
    if(K1 & K2)
      return false;

    if(K1)
    {
      if(K1 & CLIPLEFT)
      {
        point1.y += (topLeft.x - point1.x) * dy / dx;
        point1.x = topLeft.x;
      }
      else if(K1 & CLIPRIGHT)
      {
        point1.y += (bottomRight.x - point1.x) * dy / dx;
        point1.x = bottomRight.x;
      }
      if(K1 & CLIPLOWER)
      {
        point1.x += (topLeft.y - point1.y) * dx / dy;
        point1.y = topLeft.y;
      }
      else if(K1 & CLIPUPPER)
      {
        point1.x += (bottomRight.y - point1.y) * dx / dy;
        point1.y = bottomRight.y;
      }
      K1 = 0;

      if(point1.y < topLeft.y)     K1  = CLIPLOWER;
      if(point1.y > bottomRight.y) K1  = CLIPUPPER;
      if(point1.x < topLeft.x)     K1 |= CLIPLEFT;
      if(point1.x > bottomRight.x) K1 |= CLIPRIGHT;
    }

    if(K1 & K2)
      return false;

    if(K2)
    {
      if(K2 & CLIPLEFT)
      {
        point2.y += (topLeft.x - point2.x) * dy / dx;
        point2.x = topLeft.x;
      }
      else if(K2 & CLIPRIGHT)
      {
        point2.y += (bottomRight.x - point2.x) * dy / dx;
        point2.x = bottomRight.x;
      }
      if(K2 & CLIPLOWER)
      {
        point2.x += (topLeft.y - point2.y) * dx / dy;
        point2.y = topLeft.y;
      }
      else if(K2 & CLIPUPPER)
      {
        point2.x += (bottomRight.y - point2.y) * dx / dy;
        point2.y = bottomRight.y;
      }
      K2 = 0;

      if(point2.y < topLeft.y)     K2  = CLIPLOWER;
      if(point2.y > bottomRight.y) K2  = CLIPUPPER;
      if(point2.x < topLeft.x)     K2 |= CLIPLEFT;
      if(point2.x > bottomRight.x) K2 |= CLIPRIGHT;
    }
  }
  return true;
}

int Geometry::intersection(int a1, int b1, int a2, int b2, int value)
{
  int result = 0 ;
  if(a2 - a1 != 0)
    result = (int)(b1 + (float)(value - a1) / (a2 - a1) * (b2 - b1));
  else
    result = 32767;
  return(result);
}

Vector2<> Geometry::relative2FieldCoord(const Pose2D& rp, float x, float y)
{
  return rp * Vector2<>(x, y);
}

Vector2<> Geometry::relative2FieldCoord(const Pose2D& rp, const Vector2<>& relPosOnField)
{
  return rp * relPosOnField;
}

Vector2<> Geometry::fieldCoord2Relative(const Pose2D& robotPose, const Vector2<>& fieldCoord)
{
  //return robotPose.invert() * fieldCoord;

  const float invRotation = -robotPose.rotation;
  const float s = sin(invRotation);
  const float c = cos(invRotation);
  const float x = robotPose.translation.x;
  const float y = robotPose.translation.y;
  return Vector2<>(c * (fieldCoord.x - x) - s * (fieldCoord.y - y), s * (fieldCoord.x - x) + c * (fieldCoord.y - y));
}


bool Geometry::calculateBallInImage(const Vector2<>& ballOffset,
                                    const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, float ballRadius, Circle& circle)
{
  Vector2<> offset(ballOffset.x - cameraMatrix.translation.x,
                   ballOffset.y - cameraMatrix.translation.y);
  float distance = offset.abs(),
        height = cameraMatrix.translation.z - ballRadius,
        cameraDistance = sqrt(sqr(distance) + sqr(height));
  circle.center = Vector2<>(atan2(offset.y, offset.x), -atan2(height, distance));
  if(cameraDistance >= ballRadius)
  {
    float alpha = pi_2 - circle.center.y - acos(ballRadius / cameraDistance),
          yBottom = -atan2(height + cos(alpha) * ballRadius,
                           distance - sin(alpha) * ballRadius),
                    beta = pi_2 - circle.center.y + acos(ballRadius / cameraDistance),
                    yTop = -atan2(height + cos(beta) * ballRadius,
                                  distance - sin(beta) * ballRadius);
    Vector2<int> top,
            bottom;
    calculatePointByAngles(Vector2<>(circle.center.x, yTop), cameraMatrix, cameraInfo, top);
    calculatePointByAngles(Vector2<>(circle.center.x, yBottom), cameraMatrix, cameraInfo, bottom);
    circle.center.x = (top.x + bottom.x) / 2.0f;
    circle.center.y = (top.y + bottom.y) / 2.0f;
    circle.radius = (top - bottom).abs() / 2.0f;
    return true;
  }
  else
    return false;
}

float Geometry::angleSizeToPixelSize(float angleSize, const CameraInfo& cameraInfo)
{
  return cameraInfo.focalLength * tan(angleSize);
}

float Geometry::pixelSizeToAngleSize(float pixelSize, const CameraInfo& cameraInfo)
{
  return atan(pixelSize * cameraInfo.focalLengthInv);
}

float Geometry::getDistanceBySize
(
  const CameraInfo& cameraInfo,
  float sizeInReality,
  float sizeInPixels
)
{
  float xFactor = cameraInfo.focalLength;
  return sizeInReality * xFactor / (sizeInPixels + 0.000001f);
}

float Geometry::getDistanceBySize
(
  const CameraInfo& cameraInfo,
  float sizeInReality,
  float sizeInPixels,
  float centerX,
  float centerY
)
{
  float mx = centerX;
  float my = centerY;
  float cx = cameraInfo.opticalCenter.x;
  float cy = cameraInfo.opticalCenter.y;
  float focalLenPow2 = cameraInfo.focalLenPow2;
  float sqrImgRadius = (mx - cx) * (mx - cx) + (my - cy) * (my - cy);
  float imgDistance = sqrt(focalLenPow2 + sqrImgRadius);
  return imgDistance * sizeInReality / (sizeInPixels + 0.000001f);
}

float Geometry::getDistanceByAngleSize
(
  float sizeInReality,
  float sizeAsAngle
)
{
  return (sizeInReality / 2.0f) / tan(sizeAsAngle / 2.0f + 0.000001f);
}

float Geometry::getBallDistanceByAngleSize
(
  float sizeInReality,
  float sizeAsAngle
)
{
  return (sizeInReality / 2.0f) / sin(sizeAsAngle / 2.0f + 0.000001f);
}

float Geometry::getSizeByDistance
(
  const CameraInfo& cameraInfo,
  float sizeInReality,
  float distance
)
{
  float xFactor = cameraInfo.focalLength;
  return sizeInReality / distance * xFactor;
}


float Geometry::getSizeByDistance
(
  float sizeInReality,
  float distance,
  float imageWidthPixels,
  float imageWidthAngle
)
{
  float xFactor = imageWidthPixels / tan(imageWidthAngle / 2.0f) / 2.0f;
  return sizeInReality / distance * xFactor;
}

Geometry::Line Geometry::calculateHorizon
(
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo
)
{
  Line horizon;
  float r31 = cameraMatrix.rotation.c0.z;
  float r32 = cameraMatrix.rotation.c1.z;
  float r33 = cameraMatrix.rotation.c2.z;

  if(r33 == 0)
    r33 = 0.00001f;

  float x1 = 0,
        x2 = float(cameraInfo.width - 1),
        v1 = cameraInfo.focalLength,
        v2 = cameraInfo.opticalCenter.x,
        v3 = cameraInfo.opticalCenter.y,
        y1 = (v3 * r33 + r31 * v1 + r32 * v2) / r33,
        y2 = (v3 * r33 + r31 * v1 - r32 * v2) / r33;

  // Mirror ends of horizon if Camera rotated to the left
  if((cameraMatrix.rotation * Vector3<>(0, 0, 1)).z < 0)
  {
    float t = x1;
    x1 = x2;
    x2 = t;
    t = y1;
    y1 = y2;
    y2 = t;
  }

  horizon.base.x = (x1 + x2) / 2.0f;
  horizon.base.y = (y1 + y2) / 2.0f;
  horizon.direction.x = x2 - x1;
  horizon.direction.y = y2 - y1;
  horizon.normalizeDirection();
  return horizon;
}


int Geometry::calculateLineSize
(
  const Vector2<int>& pointInImage,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  float fieldLinesWidth
)
{
  return calculateLineSize(pointInImage.x, pointInImage.y, cameraMatrix, cameraInfo, fieldLinesWidth);
}

int Geometry::calculateLineSize
(
  const int xImg,
  const int yImg,
  const CameraMatrix& cameraMatrix,
  const CameraInfo& cameraInfo,
  float fieldLinesWidth
)
{
  Vector2<int> pointOnField; //position on field, relative to robot
  if(Geometry::calculatePointOnField(xImg, yImg, cameraMatrix, cameraInfo, pointOnField))
  {
    int distance = (int) sqrt(sqr(cameraMatrix.translation.z) + sqr(pointOnField.abs()));
    return (int)Geometry::getSizeByDistance(cameraInfo, fieldLinesWidth, (float) distance);
  }
  else
  {
    return 0;
  }
}



bool Geometry::calculatePointInImage(const Vector3<>& pointInWorld,
                                     const CameraMatrix& cameraMatrix,
                                     const CameraInfo& cameraInfo,
                                     Vector2<int>& pointInImage)
{
  Vector2<> offset(pointInWorld.x - cameraMatrix.translation.x,
                   pointInWorld.y - cameraMatrix.translation.y);
  return Geometry::calculatePointByAngles(
           Vector2<>(atan2(offset.y, offset.x), atan2(pointInWorld.z - cameraMatrix.translation.z, offset.abs())),
           cameraMatrix, cameraInfo,
           pointInImage
         );
}

bool Geometry::calculatePointOnHorizontalPlane(const Vector2<int>& pointInImage,
    float z,
    const CameraMatrix& cameraMatrix,
    const CameraInfo& cameraInfo,
    Vector2<>& pointOnPlane)
{
  float xFactor = cameraInfo.focalLengthInv,
        yFactor = cameraInfo.focalLengthInv;

  Vector3<>
  vectorToCenter(1, float(cameraInfo.opticalCenter.x - pointInImage.x) * xFactor,
                 float(cameraInfo.opticalCenter.y - pointInImage.y) * yFactor);

  Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;

  float a1 = cameraMatrix.translation.x,
        a2 = cameraMatrix.translation.y,
        a3 = cameraMatrix.translation.z - z,
        b1 = vectorToCenterWorld.x,
        b2 = vectorToCenterWorld.y,
        b3 = vectorToCenterWorld.z;
  if(abs(b3) > 0.00001)
  {
    pointOnPlane.x = (a1 * b3 - a3 * b1) / b3;
    pointOnPlane.y = (a2 * b3 - a3 * b2) / b3;
    return true;
  }
  else
    return false;
}
