/**
* @file Controller/Visualization/DebugDrawing3D.h
* Declaration of class DebugDrawing3D.
*
* @author Philippe Schober
*/

#pragma once

#include <vector>

#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Eigen.h"
#include "Tools/MessageQueue/InMessage.h"
#include <SimRobotCore2.h>

class DebugDrawing3D;
class RobotConsole;

/**
* Streaming operator that reads a DebugDrawing3D from a stream.
* @param stream The stream from which is read.
* @param debugDrawing3D The DebugDrawing3D object.
* @return The stream.
*/
In& operator>>(In& stream, DebugDrawing3D& debugDrawing3D);

/**
* Streaming operator that writes a DebugDrawing3D to a stream.
* @param stream The stream to write on.
* @param debugDrawing3D The DebugDrawing3D object.
* @return The stream.
*/
Out& operator<<(Out& stream, const DebugDrawing3D& debugDrawing3D);

/**
*/
class DebugDrawing3D : public SimRobotCore2::Controller3DDrawing
{
public:
  /** Default constructor. */
  DebugDrawing3D();

  /** Copy constructor for a DebugDrawing object. */
  DebugDrawing3D(const DebugDrawing3D& other);

  /** Copy constructor for a DebugDrawing object. */
  DebugDrawing3D(const DebugDrawing3D* pDebugDrawing3D);

  /** Destructor. */
  ~DebugDrawing3D();

  /** draw the elements of this drawing. */
  virtual void draw();
  void draw2();

  /** The function empties the drawing. */
  void reset();

  /** Assignment operator.*/
  const DebugDrawing3D& operator=(const DebugDrawing3D& other);

  /** Adds the contents of another debug drawing to this one. */
  const DebugDrawing3D& operator+=(const DebugDrawing3D& other);

  /** base class for all drawing elements */
  class Element
  {
  public:
    enum {DOT, LINE, POLYGON, QUAD, SPHERE, ELLIPSOID, CYLINDER, PARTDISC, IMAGE} type;
    ColorRGBA color;
    float width;
  };

  /** Stores a quad */
  class Quad : public Element
  {
  public:
    Vector3f points[4];
    Quad() { type = QUAD; }
  };

  /** Stores a polygon */
  class Polygon : public Element
  {
  public:
    Vector3f points[3];
    Polygon() { type = POLYGON; }
  };

  /** Stores a line */
  class Line : public Element
  {
  public:
    Vector3f points[2];
    Line() { type = LINE; }
  };

  class Dot : public Element
  {
  public:
    Vector3f point;
    Dot() { type = DOT; }
  };

  class Sphere : public Element
  {
  public:
    Vector3f point;
    float radius;
    Sphere() { type = SPHERE; }
  };

  class Ellipsoid : public Element
  {
  public:
    Pose3f pose;
    Vector3f radii;
    Ellipsoid() { type = ELLIPSOID; }
  };

  class Cylinder : public Element
  {
  public:
    Vector3f point;
    Vector3f rotation;
    float baseRadius;
    float topRadius;
    float height;
    Cylinder() { type = CYLINDER; }
  };

  class PartDisc : public Element
  {
  public:
    Vector3f point;
    Vector3f rotation;
    float innerRadius;
    float outerRadius;
    float startAngle;
    float sweeptAngle;
    PartDisc() { type = PARTDISC; }
  };

  class Image3D : public Element
  {
  public:
    Vector3f point;
    Vector3f rotation;
    float width,
          height;
    Image* image;
    Image3D() : image(0) { type = IMAGE; }
    Image3D(const Image3D& other) : image(0) { *this = other; }
    ~Image3D();
    const Image3D& operator=(const Image3D& other);
  };

  /**
  * Adds a line to the debug drawing.
  * @param xStart Specifies the x-coordinate of the startpoint for the line.
  * @param yStart Specifies the y-coordinate of the startpoint for the line.
  * @param zStart Specifies the z-coordinate of the startpoint for the line.
  * @param xEnd Specifies the x-coordinate of the endpoint for the line.
  * @param yEnd Specifies the y-coordinate of the endpoint for the line.
  * @param zEnd Specifies the z-coordinate of the endpoint for the line.
  * @param width Specifies the width of the line.
  * @param color Specifies the color of the line.
  */
  void line(float xStart, float yStart, float zStart,
            float xEnd, float yEnd, float zEnd,
            float width, ColorRGBA color);

  /**
  * Adds a line to the debug drawing. The line is a solid black line with width 1.
  * @param xStart Specifies the x-coordinate of the startpoint for the line.
  * @param yStart Specifies the y-coordinate of the startpoint for the line.
  * @param zStart Specified the z-coordinate of the startpoint fot the line.
  * @param xEnd Specifies the x-coordinate of the endpoint for the line.
  * @param yEnd Specifies the y-coordinate of the endpoint for the line.
  * @param zEnd Specifies the z-coordinate of the endpoint for the line.
  */
  void line(float xStart, float yStart, float zStart, float xEnd, float yEnd, float zEnd);
  void line(Vector3f* points, float width, ColorRGBA color);

  /**
   * Adds a quad to the debug drawing.
   * @param points Points to an array of points that specifies the vertices of the quad.
   * @param width Specifies the width of the border.
   * @param color Specifies the color of the polygon.
   */
  void quad(const Vector3f* points, float width, ColorRGBA color);

  /**
  * Adds a polygon to the debug drawing.
  * @param points Points to an array of points that specifies the vertices of the polygon. Each point in the array is a Point.
  * @param width Specifies the width of the border.
  * @param color Specifies the color of the polygon.
  */
  void polygon(const Vector3f* points, float width, ColorRGBA color);

  /**
  * Adds a filled square to the debug drawing. The border of the square is a solid line with width 0.
  * The square is a 3x3 square.
  * @param v Position of the dot
  * @param w The size of the dot
  * @param color The color of the dot.
  */
  void dot(Vector3f v, float w, ColorRGBA color);

  /**
  * Adds a sphere to the debug drawing.
  * @param v Position of the sphere.
  * @param radius Radius of the sphere.
  * @param color The color of the sphere.
  */
  void sphere(Vector3f v, float radius, ColorRGBA color);

  /**
  * Adds an ellypsoid to the debug drawing.
  * @param pose Pose of the ellypsoid.
  * @param radii Radii of the ellypsoid.
  * @param color The color of the ellypsoid.
  */
  void ellypsoid(const Pose3f& pose, Vector3f radii, ColorRGBA color);

  /**
  * Adds a cylinder to the debug drawing.
  * @param v Position of the cylinder
  * @param rot Rotation of the cylinder around x/y/z axes
  * @param baseRadius Radius at the base of the cylinder
  * @param topRadius Radius at the top of the cylinder
  * @param h Height of the cylinder
  * @param color The color of the cylinder.
  */
  void cylinder(Vector3f v, Vector3f rot, float baseRadius, float topRadius, float h, ColorRGBA color);

  /**
  * Adds a partial disc to the debug drawing.
  * @param v Position of the disc
  * @param rot Rotation of the disc around x/y/z axes
  * @param innerRadius Inner Radius of the disc
  * @param outerRadius Outer radius of the disc
  * @param startAngle Angle to start the part
  * @param sweepAngle Angle to end the part
  * @param color The color of the partial disc.
  */
  void partDisc(Vector3f v, Vector3f rot, float innerRadius, float outerRadius, float startAngle, float sweepAngle, ColorRGBA color);

  /**
  * Adds an image to the debug drawing.
  * @param v Position of the image
  * @param rot Rotation of the image around x/y/z axes
  * @param w Width of the image
  * @param h Height of the height
  * @param i Pointer to the image. The image will automatically be freed.
  */
  void image(Vector3f v, Vector3f rot, float w, float h, Image* i);

  bool addShapeFromQueue(InMessage& message, Drawings3D::ShapeType shapeType, char identifier);

  bool drawn; /**< is this drawing already registered? */
  bool flip; /**< rotate drawings 180 degrees around the z-axis */
  char processIdentifier; /**< To which process does this drawing belong? */
  RobotConsole* robotConsole;
  unsigned timeStamp; /**< The time when this drawing was created. */

private:
  float scaleX,  scaleY,  scaleZ;
  float rotateX, rotateY, rotateZ;
  float transX,  transY,  transZ;

  std::vector<Line>      lines;
  std::vector<Dot>       dots;
  std::vector<Polygon>   polygons;
  std::vector<Quad>      quads;
  std::vector<Sphere>    spheres;
  std::vector<Ellipsoid> ellipsoids;
  std::vector<Cylinder>  cylinders;
  std::vector<PartDisc>  partDiscs;
  std::vector<Image3D>   images;

  char* copyImage(const Image& srcImage, int& width, int& height) const;
};
