/**
 * @file Controller/Visualization/DebugDrawing3D.h
 * Declaration of class DebugDrawing3D.
 *
 * @author Philippe Schober
 */

#pragma once

#include <vector>

#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Eigen.h"
#include "Tools/MessageQueue/InMessage.h"
#include <SimRobotCore2.h>

struct CameraImage;
class RobotConsole;

class DebugDrawing3D : public SimRobotCore2::Controller3DDrawing
{
public:
  /** base class for all drawing elements */
  struct Element
  {
    ColorRGBA color;
  };

  /** Stores a quad */
  struct Quad : public Element
  {
    std::array<Vector3f, 4> points;
  };

  /** Stores a line */
  struct Line : public Element
  {
    Vector3f start, end;
    float width;
  };

  struct Dot : public Element
  {
    Vector3f point;
    float size;
  };

  struct Sphere : public Element
  {
    Vector3f point;
    float radius;
  };

  struct Ellipsoid : public Element
  {
    Pose3f pose;
    Vector3f radii;
  };

  struct Cylinder : public Element
  {
    Vector3f point;
    Vector3f rotation;
    float baseRadius;
    float topRadius;
    float height;
  };

  struct PartDisc : public Element
  {
    Vector3f point;
    Vector3f rotation;
    float innerRadius;
    float outerRadius;
    float startAngle;
    float sweeptAngle;
  };

  struct Image3D : public Element
  {
    Vector3f point;
    Vector3f rotation;
    float width, height;
    CameraImage* cameraImage = nullptr;

    Image3D() = default;
    Image3D(const Image3D& other) { *this = other; }
    ~Image3D();

    const Image3D& operator=(const Image3D& other);
  };

  bool drawn = false; /**< is this drawing already registered? */
  bool flip = false; /**< rotate drawings 180 degrees around the z-axis */
  RobotConsole* robotConsole = nullptr;
  unsigned timestamp; /**< The time when this drawing was created. */

  DebugDrawing3D();
  DebugDrawing3D(const DebugDrawing3D& other);
  DebugDrawing3D(const DebugDrawing3D* pDebugDrawing3D);

  /** draw the elements of this drawing. */
  void draw() override;
  void draw2();

  /** The function empties the drawing. */
  void reset();

  /** Assignment operator.*/
  const DebugDrawing3D& operator=(const DebugDrawing3D& other);

  /** Adds the contents of another debug drawing to this one. */
  const DebugDrawing3D& operator+=(const DebugDrawing3D& other);

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
  void line(const Vector3f& start, const Vector3f& end, float width, ColorRGBA color);

  /**
   * Adds a quad to the debug drawing.
   * @param points Points to an array of points that specifies the vertices of the quad.
   * @param color Specifies the color of the quad.
   */
  void quad(std::array<Vector3f, 4> points, ColorRGBA color);

  /**
   * Adds a filled square to the debug drawing. The border of the square is a solid line with width 0.
   * The square is a 3x3 square.
   * @param v Position of the dot
   * @param w The size of the dot
   * @param color The color of the dot.
   */
  void dot(const Vector3f& v, float w, ColorRGBA color);

  /**
   * Adds a sphere to the debug drawing.
   * @param v Position of the sphere.
   * @param radius Radius of the sphere.
   * @param color The color of the sphere.
   */
  void sphere(const Vector3f& v, float radius, ColorRGBA color);

  /**
   * Adds an ellypsoid to the debug drawing.
   * @param pose Pose of the ellypsoid.
   * @param radii Radii of the ellypsoid.
   * @param color The color of the ellypsoid.
   */
  void ellypsoid(const Pose3f& pose, const Vector3f& radii, ColorRGBA color);

  /**
   * Adds a cylinder to the debug drawing.
   * @param v Position of the cylinder
   * @param rot Rotation of the cylinder around x/y/z axes
   * @param baseRadius Radius at the base of the cylinder
   * @param topRadius Radius at the top of the cylinder
   * @param h Height of the cylinder
   * @param color The color of the cylinder.
   */
  void cylinder(const Vector3f& v, const Vector3f& rot, float baseRadius, float topRadius, float h, ColorRGBA color);

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
  void partDisc(const Vector3f& v, const Vector3f& rot, float innerRadius, float outerRadius, float startAngle, float sweepAngle, ColorRGBA color);

  /**
   * Adds an image to the debug drawing.
   * @param v Position of the image
   * @param rot Rotation of the image around x/y/z axes
   * @param w Width of the image
   * @param h Height of the height
   * @param ci Pointer to the image. The image will automatically be freed.
   */
  void image(const Vector3f& v, const Vector3f& rot, float w, float h, CameraImage* ci);

  bool addShapeFromQueue(InMessage& message, Drawings3D::ShapeType shapeType);

private:
  Vector3f scale = Vector3f::Ones();
  Vector3f rotate = Vector3f::Zero();
  Vector3f trans = Vector3f::Zero();

  std::vector<Line>      lines;
  std::vector<Dot>       dots;
  std::vector<Quad>      quads;
  std::vector<Sphere>    spheres;
  std::vector<Ellipsoid> ellipsoids;
  std::vector<Cylinder>  cylinders;
  std::vector<PartDisc>  partDiscs;
  std::vector<Image3D>   images;

  char* copyImage(const CameraImage& srcImage, unsigned int& width, unsigned int& height) const;
};
