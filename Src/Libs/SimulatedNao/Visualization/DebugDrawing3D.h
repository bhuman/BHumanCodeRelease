/**
 * @file SimulatedNao/Visualization/DebugDrawing3D.h
 * Declaration of class DebugDrawing3D.
 *
 * @author Philippe Schober
 */

#pragma once

#include "Debugging/ColorRGBA.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/Eigen.h"
#include "Math/Pose3f.h"
#include <array>
#include <vector>

struct CameraImage;

class DebugDrawing3D
{
public:
  bool addShapeFromQueue(In& message, Drawings3D::ShapeType shapeType);

  /** base class for all drawing elements */
  struct Element
  {
    ColorRGBA color;
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

  /** Stores a quad */
  struct Quad : public Element
  {
    std::array<Vector3f, 4> points;
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

public:
  Vector3f scale = Vector3f::Ones();
  Vector3f rotate = Vector3f::Zero();
  Vector3f trans = Vector3f::Zero();
  unsigned char renderOptions = 0;

  std::vector<Line>      lines;
  std::vector<Dot>       dots;
  std::vector<Quad>      quads;
  std::vector<Sphere>    spheres;
  std::vector<Ellipsoid> ellipsoids;
  std::vector<Cylinder>  cylinders;
  std::vector<Image3D>   images;

private:
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
   * Adds a filled square to the debug drawing. The border of the square is a solid line with width 0.
   * The square is a 3x3 square.
   * @param v Position of the dot
   * @param w The size of the dot
   * @param color The color of the dot.
   */
  void dot(const Vector3f& v, float w, ColorRGBA color);

  /**
   * Adds a quad to the debug drawing.
   * @param points Points to an array of points that specifies the vertices of the quad.
   * @param color Specifies the color of the quad.
   */
  void quad(const std::array<Vector3f, 4>& points, ColorRGBA color);

  /**
   * Adds a sphere to the debug drawing.
   * @param v Position of the sphere.
   * @param radius Radius of the sphere.
   * @param color The color of the sphere.
   */
  void sphere(const Vector3f& v, float radius, ColorRGBA color);

  /**
   * Adds an ellipsoid to the debug drawing.
   * @param pose Pose of the ellipsoid.
   * @param radii Radii of the ellipsoid.
   * @param color The color of the ellipsoid.
   */
  void ellipsoid(const Pose3f& pose, const Vector3f& radii, ColorRGBA color);

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
   * Adds an image to the debug drawing.
   * @param v Position of the image
   * @param rot Rotation of the image around x/y/z axes
   * @param w Width of the image
   * @param h Height of the height
   * @param ci Pointer to the image. The image will automatically be freed.
   */
  void image(const Vector3f& v, const Vector3f& rot, float w, float h, CameraImage* ci);
};
