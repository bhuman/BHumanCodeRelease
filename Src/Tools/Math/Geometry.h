/**
* @file Tools/Math/Geometry.h
* Declares class Geometry
*
* @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
* @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
*/

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Streams/Streamable.h"
#include <cstdlib>

class CameraMatrix;
class CameraInfo;

/**
* The class Geometry defines representations for geometric objects and Methods
* for calculations with such object.
*
*/
class Geometry
{
public:

  /** Defines a circle by its center and its radius*/
  struct Circle : public Streamable
  {
    Vector2<> center;
    float radius;
    
    Circle(): radius(0) {}
    Circle(const Vector2<>& c, float r)
    {
      center = c;
      radius = r;
    }
    
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(center);
      STREAM(radius);
      STREAM_REGISTER_FINISH;
    }
  };

  /** Defines a line by two vectors*/
  struct Line : public Streamable
  {
    Vector2<> base;
    Vector2<> direction;

    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN;
      STREAM(base);
      STREAM(direction);
      STREAM_REGISTER_FINISH;
   }

    Line() = default;
    Line(const Vector2<>& base, const Vector2<>& direction) :
      base(base), direction(direction)
    {}

    Line(const Vector2<int>& base, const Vector2<>& direction) :
      direction(direction)
    {
      this->base.x = (float) base.x;
      this->base.y = (float) base.y;
    }

    Line(const Vector2<int>& base, const Vector2<int>& direction)
    {
      this->base.x = (float) base.x;
      this->base.y = (float) base.y;
      this->direction.x = (float) direction.x;
      this->direction.y = (float) direction.y;
    }

    Line(const Pose2D& base, float length = 1.f)
    {
      this->base = base.translation;
      this->direction = (Pose2D(base.rotation) + Pose2D(Vector2<>(length, 0))).translation;
    }

    Line(float baseX, float baseY, float directionX, float directionY)
    {
      base.x = baseX;
      base.y = baseY;
      direction.x = directionX;
      direction.y = directionY;
    }

    void normalizeDirection();
  };

  struct PixeledLine
  {
    PixeledLine(int x1, int x2, int y1, int y2) :
      x1(x1), y1(y1), x2(x2), y2(y2)
    {
      calculatePixels();
    }

    PixeledLine(const Vector2<int>& start, const Vector2<int>& end) :
      x1(start.x), y1(start.y), x2(end.x), y2(end.y)
    {
      calculatePixels();
    }

    void calculatePixels();

    int getNumberOfPixels() const
    {
      return numberOfPixels;
    }

    int getPixelX(int i) const
    {
      return xCoordinate[i];
    }

    int getPixelY(int i) const
    {
      return yCoordinate[i];
    }

  private:
    int x1, y1, x2, y2;
    int numberOfPixels;
    enum {maxNumberOfPixelsInLine = 1600}; // max diagonal size in 4:3 image
    int xCoordinate[maxNumberOfPixelsInLine];
    int yCoordinate[maxNumberOfPixelsInLine];
  };

  /**
  * Calculates the angle between a pose and a position
  * @param from The base pose.
  * @param to The other position.
  * @return the angle from the pose to the position.
  */
  static float angleTo(const Pose2D& from,
                       const Vector2<>& to);

  /**
  * Returns the circle defined by the three points.
  * @param point1 The first point.
  * @param point2 The second point.
  * @param point3 The third point.
  * @return The circle defined by point1, point2 and point3.
  */
  static Circle getCircle(
    const Vector2<int>& point1,
    const Vector2<int>& point2,
    const Vector2<int>& point3
  );

  static int getIntersectionOfCircles(
    const Circle& c1,
    const Circle& c2,
    Vector2<>& p1,
    Vector2<>& p2
  );

  /**
  * Computes the intersection point of a line and a circle.
  * @param line The Line.
  * @param circle The Circle.
  * @param firstIntersection The first intersection point, if there is one.
  * @param secondIntersection The second intersection point, if there is one.
  * @return The number of intersection points.
  */
  static int getIntersectionOfLineAndCircle(
    const Line& line,
    const Circle& circle,
    Vector2<>& firstIntersection,
    Vector2<>& secondIntersection
  );

  static bool checkIntersectionOfLines(
    const Vector2<>& l1p1,
    const Vector2<>& l1p2,
    const Vector2<>& l2p1,
    const Vector2<>& l2p2
  );

  static bool getIntersectionOfLines(
    const Line& line1,
    const Line& line2,
    Vector2<>& intersection
  );

  static bool getIntersectionOfRaysFactor(
    const Line& ray1,
    const Line& ray2,
    float& intersection
  );

  static float getDistanceToLine(
    const Line& line,
    const Vector2<>& point
  );

  static float getDistanceToEdge(
    const Line& line,
    const Vector2<>& point
  );

  static float distance(
    const Vector2<>& point1,
    const Vector2<>& point2
  );

  static float distance(
    const Vector2<int>& point1,
    const Vector2<int>& point2
  );

private:
  static int ccw(
    const Vector2<>& p0,
    const Vector2<>& p1,
    const Vector2<>& p2);

public:
  static void calculateAnglesForPoint(
    const Vector2<>& point,
    const CameraMatrix& cameraMatrix,
    const CameraInfo& cameraInfo,
    Vector2<>& angles
  );

  static bool calculatePointByAngles(
    const Vector2<>& angles,
    const CameraMatrix& cameraMatrix,
    const CameraInfo& cameraInfo,
    Vector2<float>& point
  );

  static bool clipLineWithQuadrangle(
    const Line& lineToClip,
    const Vector2<>& corner0,
    const Vector2<>& corner1,
    const Vector2<>& corner2,
    const Vector2<>& corner3,
    Vector2<>& clipPoint1,
    Vector2<>& clipPoint2
  );

  static bool clipLineWithQuadrangle(
    const Line& lineToClip,
    const Vector2<>& corner0,
    const Vector2<>& corner1,
    const Vector2<>& corner2,
    const Vector2<>& corner3,
    Vector2<int>& clipPoint1,
    Vector2<int>& clipPoint2
  );

  static bool isPointInsideRectangle(
    const Vector2<>& bottomLeftCorner,
    const Vector2<>& topRightCorner,
    const Vector2<>& point
  );

  static bool isPointInsideRectangle2(
    const Vector2<>& corner1,
    const Vector2<>& corner2,
    const Vector2<>& point
  );

  static bool isPointInsideRectangle(
    const Vector2<int>& bottomLeftCorner,
    const Vector2<int>& topRightCorner,
    const Vector2<int>& point
  );

  static bool isPointInsideConvexPolygon(
    const Vector2<> polygon[],
    const int numberOfPoints,
    const Vector2<>& point
  );

  static bool clipPointInsideRectangle(
    const Vector2<int>& bottomLeftCorner,
    const Vector2<int>& topRightCorner,
    Vector2<int>& point
  );

  static bool clipPointInsideRectangle(
    const Vector2<int>& bottomLeftCorner,
    const Vector2<int>& topRightCorner,
    Vector2<>& point
  );

  /**
  * Clips a line with a rectangle
  * @param bottomLeft The bottom left corner of the rectangle
  * @param topRight The top right corner of the rectangle
  * @param line The line to be clipped
  * @param point1 The starting point of the resulting line
  * @param point2 The end point of the resulting line
  * @return states whether clipping was necessary (and done)
  */
  static bool getIntersectionPointsOfLineAndRectangle(
    const Vector2<int>& bottomLeft,
    const Vector2<int>& topRight,
    const Geometry::Line& line,
    Vector2<int>& point1,
    Vector2<int>& point2
  );

  static bool getIntersectionPointsOfLineAndRectangle(
    const Vector2<>& bottomLeft,
    const Vector2<>& topRight,
    const Geometry::Line& line,
    Vector2<>& point1,
    Vector2<>& point2
  );

  /**
  * Clips a line with the Cohen-Sutherland-Algorithm
  * @param bottomLeft The bottom left corner of the rectangle
  * @param topRight The top right corner of the rectangle
  * @param point1 The starting point of the line
  * @param point2 The end point of the line
  * @return states whether line exists after clipping
  * @see http://de.wikipedia.org/wiki/Algorithmus_von_Cohen-Sutherland
  */
  static bool clipLineWithRectangleCohenSutherland(
    const Vector2<int>& bottomLeft,
    const Vector2<int>& topRight,
    Vector2<int>& point1,
    Vector2<int>& point2
  );

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
  static bool calculateBallInImage(const Vector2<>& ballOffset,
                                   const CameraMatrix& cameraMatrix,
                                   const CameraInfo& cameraInfo,
                                   float ballRadius,
                                   Circle& circle);

  /**
  * The function determines how far an object is away depending on its real size and the size in the image.
  * @param cameraInfo Information about the camera (opening angles, resolution, etc.).
  * @param sizeInReality The real size of the object.
  * @param sizeInPixels The size in the image.
  * @return The distance between camera and object.
  */
  static float getDistanceBySize
  (
    const CameraInfo& cameraInfo,
    float sizeInReality,
    float sizeInPixels
  );

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
  static float getDistanceBySize(
    const CameraInfo& cameraInfo,
    float sizeInReality,
    float sizeInPixels,
    float centerX,
    float centerY
  );

  /**
  * The function determines how big an object appears in the image depending on its distance and size.
  * @param cameraInfo Object containing camera paramters.
  * @param sizeInReality The real size of the object.
  * @param distance The distance to the object.
  * @return The size as it would appear in the image.
  */
  static float getSizeByDistance
  (
    const CameraInfo& cameraInfo,
    float sizeInReality,
    float distance
  );

  /**
  * The function calculates the horizon.
  * @param cameraMatrix The camera matrix.
  * @param cameraInfo Object containing camera parameters.
  * @return The line of the horizon in the image.
  */
  static Geometry::Line calculateHorizon(
    const CameraMatrix& cameraMatrix,
    const CameraInfo& cameraInfo
  );

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
  static void linspaced(const Vector2<>& start, const Vector2<>& stop, unsigned count, std::vector<Vector2<>>& out);
};
