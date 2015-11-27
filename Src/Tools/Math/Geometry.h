/**
 * @file Tools/Math/Geometry.h
 * Declares class Geometry
 *
 * @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
 * @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"

struct CameraMatrix;
struct CameraInfo;
struct FieldDimensions;
struct RobotPose;

/**
 * The class Geometry defines representations for geometric objects and Methods
 * for calculations with such object.
 */
class Geometry
{
public:
  /** Defines a circle by its center and its radius*/
  STREAMABLE(Circle,
  {
    Circle() = default;
    Circle(const Vector2f& c, float r),

    (Vector2f)(Vector2f::Zero()) center,
    (float)(0) radius,
  });

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

  struct PixeledLine : public std::vector<Vector2i>
  {
    PixeledLine(int x1, int y1, int x2, int y2)
    {
      calculatePixels(x1, y1, x2, y2);
    }

    PixeledLine(const Vector2i& start, const Vector2i& end)
    {
      calculatePixels(start.x(), start.y(), end.x(), end.y());
    }

  private:
    void calculatePixels(int x1, int y1, int x2, int y2);
  };

  /**
   * Calculates the angle between a pose and a position
   * @param from The base pose.
   * @param to The other position.
   * @return the angle from the pose to the position.
   */
  static float angleTo(const Pose2f& from, const Vector2f& to);

  /**
   * Returns the circle defined by the three points.
   * @param point1 The first point.
   * @param point2 The second point.
   * @param point3 The third point.
   * @return The circle defined by point1, point2 and point3.
   */
  static Circle getCircle(const Vector2i& point1, const Vector2i& point2, const Vector2i& point3);
  static int getIntersectionOfCircles(const Circle& c1, const Circle& c2, Vector2f& p1, Vector2f& p2);

  /**
   * Computes the intersection point of a line and a circle.
   * @param line The Line.
   * @param circle The Circle.
   * @param firstIntersection The first intersection point, if there is one.
   * @param secondIntersection The second intersection point, if there is one.
   * @return The number of intersection points.
   */
  static int getIntersectionOfLineAndCircle(const Line& line, const Circle& circle, Vector2f& firstIntersection, Vector2f& secondIntersection);
  static bool checkIntersectionOfLines(const Vector2f& l1p1, const Vector2f& l1p2, const Vector2f& l2p1, const Vector2f& l2p2);
  static bool getIntersectionOfLines(const Line& line1, const Line& line2, Vector2f& intersection) WARN_UNUSED_RESULT;
  static bool getIntersectionOfRaysFactor(const Line& ray1, const Line& ray2, float& intersection) WARN_UNUSED_RESULT;

  static float getDistanceToLine(const Line& line, const Vector2f& point);
  static float getDistanceToEdge(const Line& line, const Vector2f& point);

  static float distance(const Vector2f& point1, const Vector2f& point2);
  static float distance(const Vector2i& point1, const Vector2i& point2);

private:
  static int ccw(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2);

public:
  static void calculateAnglesForPoint(const Vector2f& point, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& angles);
  static bool calculatePointByAngles(const Vector2f& angles, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& point) WARN_UNUSED_RESULT;
  static bool isPointInsideRectangle(const Vector2f& bottomLeftCorner, const Vector2f& topRightCorner, const Vector2f& point) WARN_UNUSED_RESULT;
  static bool isPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, const Vector2i& point) WARN_UNUSED_RESULT;
  static bool isPointInsideRectangle2(const Vector2f& corner1, const Vector2f& corner2, const Vector2f& point);
  static bool isPointInsideConvexPolygon(const Vector2f polygon[], const int numberOfPoints, const Vector2f& point) WARN_UNUSED_RESULT;
  static bool clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2i& point) WARN_UNUSED_RESULT;
  static bool clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2f& point) WARN_UNUSED_RESULT;

  /**
   * Clips a line with a rectangle
   * @param bottomLeft The bottom left corner of the rectangle
   * @param topRight The top right corner of the rectangle
   * @param line The line to be clipped
   * @param point1 The starting point of the resulting line
   * @param point2 The end point of the resulting line
   * @return states whether clipping was necessary (and done)
   */
  static bool getIntersectionPointsOfLineAndRectangle(const Vector2i& bottomLeft, const Vector2i& topRight,
      const Geometry::Line& line, Vector2i& point1, Vector2i& point2) WARN_UNUSED_RESULT;
  static bool getIntersectionPointsOfLineAndRectangle(const Vector2f& bottomLeft, const Vector2f& topRight,
      const Geometry::Line& line, Vector2f& point1, Vector2f& point2) WARN_UNUSED_RESULT;

  /**
   * Clips a line with the Cohen-Sutherland-Algorithm
   * @param bottomLeft The bottom left corner of the rectangle
   * @param topRight The top right corner of the rectangle
   * @param point1 The starting point of the line
   * @param point2 The end point of the line
   * @return states whether line exists after clipping
   * @see http://de.wikipedia.org/wiki/Algorithmus_von_Cohen-Sutherland
   */
  static bool clipLineWithRectangleCohenSutherland(const Vector2i& bottomLeft, const Vector2i& topRight,
      Vector2i& point1, Vector2i& point2) WARN_UNUSED_RESULT;

  /**
   * Calculates the intersection of an arbitrary line and a horizontal or vertical line.
   */
  static int intersection(int a1, int b1, int a2, int b2, int value);

  /**
   * The function approximates the shape of a ball in the camera image.
   * Note: currently, the approximation is not exact.
   * @param ballOffset The ball's position relative to the robot's body origin.
   * @param cameraMatrix The position and orientation of the robot's camera.
   * @param cameraInfo The resolution and the opening angle of the robot's camera.
   * @param ballRadius The radius of the ball in mm.
   * @param circle The approximated shape generated by the function.
   * @return If false, only the center of the circle is valid, not the radius.
   */
  static bool calculateBallInImage(const Vector2f& ballOffset, const CameraMatrix& cameraMatrix,
                                   const CameraInfo& cameraInfo, float ballRadius, Circle& circle) WARN_UNUSED_RESULT;

  /**
   * This functions computes a polygon (of four points) described by four points.
   * This polygon describes the part of the field that can be seen in the current image.
   * All points are in field coordinates
   * @param robotPose The pose of the robot on the field
   * @param cameraMatrix The position and orientation of the robot's camera.
   * @param cameraInfo The resolution and the opening angle of the robot's camera.
   * @param p The list of points
   */
  static void computeFieldOfViewInFieldCoordinates(const RobotPose& robotPose, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo,
      const FieldDimensions& fieldDimensions, std::vector<Vector2f>& p);

  /**
   * The function determines how far an object is away depending on its real size and the size in the image.
   * @param cameraInfo Information about the camera (opening angles, resolution, etc.).
   * @param sizeInReality The real size of the object.
   * @param sizeInPixels The size in the image.
   * @return The distance between camera and object.
   */
  static float getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels);

  /**
   * The function determines how far an object is away depending on its real size and the size in the image
   * along with its center position, using camera intrinsic parameters.
   * @param cameraInfo Class containing the intrinsic paramaters
   * @param sizeInReality The real size of the object.
   * @param sizeInPixels The size in the image.
   * @param centerX X coordinate (in image reference) of object's baricenter.
   * @param centerY Y coordinate (in image reference) of object's baricenter.
   * @return The distance between camera and object.
   */
  static float getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels, float centerX, float centerY);

  /**
   * The function determines how big an object appears in the image depending on its distance and size.
   * @param cameraInfo Object containing camera parameters.
   * @param sizeInReality The real size of the object.
   * @param distance The distance to the object.
   * @return The size as it would appear in the image.
   */
  static float getSizeByDistance(const CameraInfo& cameraInfo, float sizeInReality, float distance);

  /**
   * The function calculates the horizon.
   * @param cameraMatrix The camera matrix.
   * @param cameraInfo Object containing camera parameters.
   * @return The line of the horizon in the image.
   */
  static Geometry::Line calculateHorizon(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo);

  /**
   * Calculates the angle size for a given pixel size.
   */
  static float pixelSizeToAngleSize(float pixelSize, const CameraInfo& cameraInfo);

  /**
   * Calculates the pixel size for a given angle size.
   */
  static float angleSizeToPixelSize(float angleSize, const CameraInfo& cameraInfo);

  /**
   * Returns evenly spaced samples, calculated over the interval [start, stop]
   */
  static void linspaced(const Vector2f& start, const Vector2f& stop, unsigned count, std::vector<Vector2f>& out);

  /**
   * @param x1, y2, x2, y2, x3, y3 Points of the triangle
   * @param px, py the point that will be tested
   * @return true if (px, py) is inside the triangle defined by (x1, y1, x2, y2, x3, y3)
   */
  static bool isPointInsideTriangle(const float x1, const float y1, const float x2, const float y2,
                                    const float x3, const float y3, const float px, const float py);

  /**
   * Computes the difference between two angles (considering the interval [-pi,..,pi])
   * @param x The first angle
   * @param y The second angle
   * @return The absolute difference
   */
  static float absDifferenceBetweenTwoAngles(float x, float y)
  {
    return std::abs(std::atan2(std::sin(x - y), std::cos(x - y)));
  }
};

inline Geometry::Circle::Circle(const Vector2f& c, float r) :
  center(c), radius(r)
{}

inline Geometry::Line::Line(const Vector2f& base, const Vector2f& direction) :
  base(base), direction(direction)
{}

inline Geometry::Line::Line(const Vector2i& base, const Vector2f& direction) :
  base(static_cast<float>(base.x()), static_cast<float>(base.y())),
  direction(direction)
{}

inline Geometry::Line::Line(const Vector2i& base, const Vector2i& direction) :
  base(static_cast<float>(base.x()), static_cast<float>(base.y())),
  direction(static_cast<float>(direction.x()), static_cast<float>(direction.y()))
{}

inline Geometry::Line::Line(const Pose2f& base, float length) :
  base(base.translation),
  direction(Pose2f(base.rotation) * Vector2f(length, 0))
{}

inline Geometry::Line::Line(float baseX, float baseY, float directionX, float directionY) :
  base(baseX, baseY), direction(directionX, directionY)
{}
