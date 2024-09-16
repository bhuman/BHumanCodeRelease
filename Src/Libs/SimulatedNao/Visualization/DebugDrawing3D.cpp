/**
 * @file SimulatedNao/Visualization/DebugDrawing3D.cpp
 * Implementation of class DebugDrawing3D.
 *
 * @author Philippe Schober
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "DebugDrawing3D.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/CameraImage.h"

bool DebugDrawing3D::addShapeFromQueue(In& stream, Drawings3D::ShapeType shapeType)
{
  switch(shapeType)
  {
    case Drawings3D::translate:
    {
      stream >> trans.x();
      stream >> trans.y();
      stream >> trans.z();
      break;
    }
    case Drawings3D::scale:
    {
      stream >> scale.x();
      stream >> scale.y();
      stream >> scale.z();
      break;
    }
    case Drawings3D::rotate:
    {
      stream >> rotate.x();
      stream >> rotate.y();
      stream >> rotate.z();
      break;
    }
    case Drawings3D::coordinates:
    {
      float width;
      float length;
      stream >> length;
      stream >> width;
      this->line(0, 0, 0, length, 0, 0, width, ColorRGBA::red);
      this->line(0, 0, 0, 0, length, 0, width, ColorRGBA::green);
      this->line(0, 0, 0, 0, 0, length, width, ColorRGBA::blue);
      break;
    }
    case Drawings3D::quad:
    {
      std::array<Vector3f, 4> points;
      ColorRGBA c;
      stream >> points[0];
      stream >> points[1];
      stream >> points[2];
      stream >> points[3];
      stream >> c;
      this->quad(points, c);
      break;
    }
    case Drawings3D::line:
    {
      Vector3f start, end;
      float width;
      ColorRGBA c;
      stream >> start;
      stream >> end;
      stream >> width;
      stream >> c;
      this->line(start, end, width, c);
      break;
    }
    case Drawings3D::cube:
    {
      Vector3f points[8];
      float width;
      ColorRGBA c;
      for(int i = 0; i < 8; i++)
        stream >> points[i];

      stream >> width;
      stream >> c;
      this->line(points[0], points[1], width, c); //AB
      this->line(points[0], points[2], width, c); //AC
      this->line(points[0], points[4], width, c); //AE
      this->line(points[1], points[3], width, c); //BD
      this->line(points[1], points[5], width, c); //BF
      this->line(points[2], points[3], width, c); //CD
      this->line(points[2], points[6], width, c); //CG
      this->line(points[3], points[7], width, c); //DH
      this->line(points[4], points[6], width, c); //EG
      this->line(points[4], points[5], width, c); //EF
      this->line(points[5], points[7], width, c); //FH
      this->line(points[6], points[7], width, c); //GH
      break;
    }
    case Drawings3D::dot:
    {
      Vector3f v;
      float w;
      ColorRGBA c;
      bool withLines;
      stream >> v;
      stream >> w;
      stream >> c;
      stream >> withLines;
      this->dot(v, w, c);

      if(withLines)
      {
        this->line(0, 0, 0, v.x(), 0, 0, 1.0f, c);
        this->line(0, 0, 0, 0, v.y(), 0, 1.0f, c);
        this->line(0, 0, 0, 0, 0, v.z(), 1.0f, c);
        this->line(v.x(), 0, 0, v.x(), v.y(), 0, 1.0f, c);
        this->line(v.x(), 0, 0, v.x(), 0, v.z(), 1.0f, c);
        this->line(0, v.y(), 0, 0, v.y(), v.z(), 1.0f, c);
        this->line(v.x(), v.y(), 0, 0, v.y(), 0, 1.0f, c);
        this->line(v.x(), v.y(), 0, v.x(), v.y(), v.z(), 1.0f, c);
        this->line(v.x(), 0, v.z(), v.x(), v.y(), v.z(), 1.0f, c);
        this->line(v.x(), 0, v.z(), 0, 0, v.z(), 1.0f, c);
        this->line(0, v.y(), v.z(), 0, 0, v.z(), 1.0f, c);
        this->line(0, v.y(), v.z(), v.x(), v.y(), v.z(), 1.0f, c);
      }
      break;
    }
    case Drawings3D::sphere:
    {
      Vector3f v;
      float r;
      ColorRGBA c;
      stream >> v;
      stream >> r;
      stream >> c;
      this->sphere(v, r, c);
      break;
    }
    case Drawings3D::ellipsoid:
    {
      Pose3f p;
      Vector3f s;
      ColorRGBA c;
      stream >> p;
      stream >> s;
      stream >> c;
      this->ellipsoid(p, s, c);
      break;
    }
    case Drawings3D::cylinder:
    {
      Vector3f v, rot;
      float r, r2, h;
      ColorRGBA c;
      stream >> v;
      stream >> rot;
      stream >> r;
      stream >> r2;
      stream >> h;
      stream >> c;
      this->cylinder(v, rot, r, r2, h, c);
      break;
    }
    case Drawings3D::image:
    {
      Vector3f v, rot;
      float w, h;
      CameraImage* ci = new CameraImage;
      stream >> v;
      stream >> rot;
      stream >> w;
      stream >> h;
      stream >> *ci;
      this->image(v, rot, w, h, ci);
      break;
    }
    case Drawings3D::renderOptions:
    {
      stream >> renderOptions;
      break;
    }
  }
  return true;
}

void DebugDrawing3D::line(float xStart, float yStart, float zStart,
                          float xEnd, float yEnd, float zEnd,
                          float width, ColorRGBA color)
{
  line(Vector3f(xStart, yStart, zStart), Vector3f(xEnd, yEnd, zEnd), width, color);
}

void DebugDrawing3D::line(const Vector3f& start, const Vector3f& end, float width, ColorRGBA color)
{
  Line element;
  element.start = start;
  element.end = end;
  element.color = color;
  element.width = width;
  lines.push_back(element);
}

void DebugDrawing3D::dot(const Vector3f& v, float w, ColorRGBA color)
{
  Dot element;
  element.point = v;
  element.color = color;
  element.size = w;
  dots.push_back(element);
}

void DebugDrawing3D::quad(const std::array<Vector3f, 4>& points, ColorRGBA color)
{
  Quad element;
  element.points = points;
  element.color = color;
  quads.push_back(element);
}

void DebugDrawing3D::sphere(const Vector3f& v, float r, ColorRGBA color)
{
  Sphere element;
  element.point = v;
  element.color = color;
  element.radius = r;
  spheres.push_back(element);
}

void DebugDrawing3D::ellipsoid(const Pose3f& pose, const Vector3f& radii, ColorRGBA color)
{
  Ellipsoid element;
  element.pose = pose;
  element.radii = radii;
  element.color = color;
  ellipsoids.push_back(element);
}

void DebugDrawing3D::cylinder(const Vector3f& v, const Vector3f& rot, float baseRadius,
                              float topRadius, float h, ColorRGBA color)
{
  Cylinder element;
  element.point = v;
  element.rotation = rot;
  element.color = color;
  element.baseRadius = baseRadius;
  element.topRadius = topRadius;
  element.height = h;
  cylinders.push_back(element);
}

void DebugDrawing3D::image(const Vector3f& v, const Vector3f& rot, float w, float h, CameraImage* ci)
{
  Image3D element;
  element.point = v;
  element.rotation = rot;
  element.cameraImage = ci;
  element.width = w;
  element.height = h;
  images.push_back(element);
}

DebugDrawing3D::Image3D::~Image3D()
{
  delete cameraImage;
}

const DebugDrawing3D::Image3D& DebugDrawing3D::Image3D::operator=(const DebugDrawing3D::Image3D& other)
{
  ASSERT(this != &other);
  delete cameraImage;

  *static_cast<Element*>(this) = other;
  point = other.point;
  rotation = other.rotation;
  width = other.width;
  height = other.height;
  cameraImage = other.cameraImage;

  // dirty hack: works only for the current use of this class
  const_cast<Image3D&>(other).cameraImage = nullptr;
  return *this;
}
