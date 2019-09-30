/**
 * @file Controller/Visualization/DebugDrawing3D.cpp
 * Implementation of class DebugDrawing3D.
 *
 * @author Philippe Schober
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "Controller/RobotConsole.h"
#ifdef MACOS
#include <gl.h>
#include <glu.h>
#else
#include <GL/glew.h>
#endif
#include "DebugDrawing3D.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"

DebugDrawing3D::DebugDrawing3D() : timestamp(Time::getCurrentSystemTime()) {}

const DebugDrawing3D& DebugDrawing3D::operator=(const DebugDrawing3D& other)
{
  reset();
  timestamp = other.timestamp;
  drawn = other.drawn;
  flip = other.flip;
  scale = other.scale;
  rotate = other.rotate;
  trans = other.trans;
  robotConsole = other.robotConsole;
  return *this += other;
}

const DebugDrawing3D& DebugDrawing3D::operator+=(const DebugDrawing3D& other)
{
  lines.insert(lines.end(), other.lines.begin(), other.lines.end());
  dots.insert(dots.end(), other.dots.begin(), other.dots.end());
  quads.insert(quads.end(), other.quads.begin(), other.quads.end());
  spheres.insert(spheres.end(), other.spheres.begin(), other.spheres.end());
  ellipsoids.insert(ellipsoids.end(), other.ellipsoids.begin(), other.ellipsoids.end());
  cylinders.insert(cylinders.end(), other.cylinders.begin(), other.cylinders.end());
  partDiscs.insert(partDiscs.end(), other.partDiscs.begin(), other.partDiscs.end());
  images.insert(images.end(), other.images.begin(), other.images.end());
  return *this;
}

DebugDrawing3D::DebugDrawing3D(const DebugDrawing3D& other)
{
  *this = other;
}

DebugDrawing3D::DebugDrawing3D(const DebugDrawing3D* pDebugDrawing3D)
{
  *this = *pDebugDrawing3D;
}

void DebugDrawing3D::draw()
{
  if(robotConsole)
  {
    SYNC_WITH(*robotConsole);
    draw2();
  }
  else
    draw2();
}

void DebugDrawing3D::draw2()
{
  glPushAttrib(GL_ENABLE_BIT);
  glPushMatrix();

  if(flip)
    glRotated(180., 0., 0., 1.);

  // Convert mm to m.
  glScaled(0.001, 0.001, 0.001);

  // Custom scaling.
  glScalef(scale.x(), scale.y(), scale.z());

  // Custom rotation.
  if(rotate.z() != 0)
    glRotatef(toDegrees(rotate.z()), 0., 0., 1.);
  if(rotate.y() != 0)
    glRotatef(toDegrees(rotate.y()), 0., 1., 0.);
  if(rotate.x() != 0)
    glRotatef(toDegrees(rotate.x()), 1., 0., 0.);

  // Custom translation.
  glTranslatef(trans.x(), trans.y(), trans.z());

  glEnable(GL_NORMALIZE);

  // Draw all lines.
  if(!lines.empty())
  {
    glPushAttrib(GL_LINE_BIT);
    for(const Line& l : lines)
    {
      glLineWidth(l.width);
      glColor4ub(l.color.r, l.color.g, l.color.b, l.color.a);
      glBegin(GL_LINES);
      glVertex3f(l.start.x(), l.start.y(), l.start.z());
      glVertex3f(l.end.x(), l.end.y(), l.end.z());
      glEnd();
    }
    glPopAttrib();
  }

  // Draw all points.
  if(!dots.empty())
  {
    glPushAttrib(GL_POINT_BIT);
    for(const Dot& d : dots)
    {
      // Since each point may have a different size we can't handle all
      // points in a single glBegin(GL_POINTS).
      // ( glPointSize is not allowed in a glBegin(...). )
      glPointSize(d.size);
      glColor4ub(d.color.r, d.color.g, d.color.b, d.color.a);
      glBegin(GL_POINTS);
      glVertex3f(d.point.x(), d.point.y(), d.point.z());
      glEnd();
    }
    glPopAttrib();
  }

  // draw spheres
  for(const Sphere& s : spheres)
  {
    glColor4ub(s.color.r, s.color.g, s.color.b, s.color.a);
    glPushMatrix();
    glTranslatef(s.point.x(), s.point.y(), s.point.z());
    GLUquadric* q = gluNewQuadric();
    gluSphere(q, s.radius, 16, 16);
    gluDeleteQuadric(q);
    glPopMatrix();
  }

  // draw ellypsoids
  for(const Ellipsoid& e : ellipsoids)
  {
    glColor4ub(e.color.r, e.color.g, e.color.b, e.color.a);
    glPushMatrix();
    glTranslatef(e.pose.translation.x(), e.pose.translation.y(), e.pose.translation.z());
    AngleAxisf aa(e.pose.rotation);
    glRotatef(toDegrees(aa.angle()), aa.axis().x(), aa.axis().y(), aa.axis().z());
    glScalef(e.radii.x(), e.radii.y(), e.radii.z());
    GLUquadric* q = gluNewQuadric();
    gluSphere(q, 1, 16, 16);
    gluDeleteQuadric(q);
    glPopMatrix();
  }

  // Draw all quads.
  for(const Quad& q : quads)
  {
    glColor4ub(q.color.r, q.color.g, q.color.b, q.color.a);
    glBegin(GL_QUADS);

    const Vector3f& p1 = q.points[0];
    const Vector3f& p2 = q.points[1];
    const Vector3f& p3 = q.points[2];
    const Vector3f& p4 = q.points[3];
    Vector3f u = p2 - p1;
    Vector3f v = p3 - p1;
    Vector3f n = u.cross(v);
    n.normalize();

    glNormal3fv(&n.x());
    glVertex3fv(&p1.x());
    glVertex3fv(&p2.x());
    glVertex3fv(&p3.x());
    glVertex3fv(&p4.x());
    glEnd();
  }

  // draw cylinders
  for(const Cylinder& c : cylinders)
  {
    glColor4ub(c.color.r, c.color.g, c.color.b, c.color.a);

    glPushMatrix();
    glTranslated(c.point.x(), c.point.y(), c.point.z());
    if(c.rotation.x() != 0)
      glRotated(toDegrees(c.rotation.x()), 1, 0, 0);
    if(c.rotation.y() != 0)
      glRotated(toDegrees(c.rotation.y()), 0, 1, 0);
    if(c.rotation.z() != 0)
      glRotated(toDegrees(c.rotation.z()), 0, 0, 1);
    glTranslated(0, 0, -c.height / 2);
    GLUquadric* q = gluNewQuadric();
    gluCylinder(q, c.baseRadius, c.topRadius, c.height, 16, 1);
    glRotated(180, 0, 1, 0);
    if(c.baseRadius > 0.f)
      gluDisk(q, 0, c.baseRadius, 16, 1);
    glRotated(180, 0, 1, 0);
    glTranslated(0, 0, c.height);
    if(c.topRadius > 0.f)
      gluDisk(q, 0, c.topRadius, 16, 1);
    gluDeleteQuadric(q);
    glPopMatrix();
  }

  for(const PartDisc& pD : partDiscs)
  {
    glColor4ub(pD.color.r, pD.color.g, pD.color.b, pD.color.a);

    glPushMatrix();
    glTranslated(pD.point.x(), pD.point.y(), pD.point.z());
    if(pD.rotation.x() != 0)
      glRotated(toDegrees(pD.rotation.x()), 1, 0, 0);
    if(pD.rotation.y() != 0)
      glRotated(toDegrees(pD.rotation.y()), 0, 1, 0);
    if(pD.rotation.z() != 0)
      glRotated(toDegrees(pD.rotation.z()), 0, 0, 1);

    GLUquadric* q = gluNewQuadric();
    gluPartialDisk(q, pD.innerRadius, pD.outerRadius, 16, 10, toDegrees(pD.startAngle), toDegrees(pD.sweeptAngle));

    gluDeleteQuadric(q);
    glPopMatrix();
  }

  // draw 3d images
  if(!images.empty())
  {
    glPushAttrib(GL_TEXTURE_BIT);
    glDisable(GL_CULL_FACE);
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    for(const Image3D& i : images)
    {
      GLuint t;
      glGenTextures(1, &t);
      glBindTexture(GL_TEXTURE_2D, t);

      unsigned int width, height;
      char* imageData = copyImage(*i.cameraImage, width, height);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, imageData);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      delete[] imageData;

      glPushMatrix();
      glTranslated(i.point.x(), i.point.y(), i.point.z());
      if(i.rotation.x() != 0)
        glRotated(toDegrees(i.rotation.x()), 1, 0, 0);
      if(i.rotation.y() != 0)
        glRotated(toDegrees(i.rotation.y()), 0, 1, 0);
      if(i.rotation.z() != 0)
        glRotated(toDegrees(i.rotation.z()), 0, 0, 1);
      glBegin(GL_QUADS);
      float right = static_cast<float>(i.cameraImage->width) / width;
      float top = static_cast<float>(i.cameraImage->height) / height;
      glTexCoord2d(right, top);
      glVertex3d(0, -i.width / 2, i.height / 2);
      glTexCoord2d(0, top);
      glVertex3d(0, i.width / 2, i.height / 2);
      glTexCoord2d(0, 0);
      glVertex3d(0, i.width / 2, -i.height / 2);
      glTexCoord2d(right, 0);
      glVertex3d(0, -i.width / 2, -i.height / 2);
      glEnd();
      glPopMatrix();
      glDeleteTextures(1, &t);
    }
    glPopAttrib();
  }

  //
  glPopAttrib();
  glPopMatrix();
}

void DebugDrawing3D::reset()
{
  timestamp = Time::getCurrentSystemTime();
  lines.clear();
  dots.clear();
  quads.clear();
  spheres.clear();
  ellipsoids.clear();
  cylinders.clear();
  partDiscs.clear();
  images.clear();
}

void DebugDrawing3D::quad(std::array<Vector3f, 4> points, ColorRGBA color)
{
  Quad element;
  element.points = points;
  element.color = color;
  quads.push_back(element);
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

void DebugDrawing3D::sphere(const Vector3f& v, float r, ColorRGBA color)
{
  Sphere element;
  element.point = v;
  element.color = color;
  element.radius = r;
  spheres.push_back(element);
}

void DebugDrawing3D::ellypsoid(const Pose3f& pose, const Vector3f& radii, ColorRGBA color)
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

void DebugDrawing3D::partDisc(const Vector3f& v, const Vector3f& rot, float innerRadius, float outerRadius,
                              float startAngle, float sweepAngle, ColorRGBA color)
{
  PartDisc element;
  element.point = v;
  element.rotation = rot;
  element.innerRadius = innerRadius;
  element.outerRadius = outerRadius;
  element.startAngle = startAngle;
  element.sweeptAngle = sweepAngle;
  element.color = color;
  partDiscs.push_back(element);
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

bool DebugDrawing3D::addShapeFromQueue(InMessage& message, Drawings3D::ShapeType shapeType)
{
  switch((Drawings3D::ShapeType)shapeType)
  {
    case Drawings3D::translate:
    {
      message.bin >> trans.x();
      message.bin >> trans.y();
      message.bin >> trans.z();
      break;
    }
    case Drawings3D::scale:
    {
      message.bin >> scale.x();
      message.bin >> scale.y();
      message.bin >> scale.z();
      break;
    }
    case Drawings3D::rotate:
    {
      message.bin >> rotate.x();
      message.bin >> rotate.y();
      message.bin >> rotate.z();
      break;
    }
    case Drawings3D::coordinates:
    {
      float width;
      float length;
      message.bin >> length;
      message.bin >> width;
      this->line(0, 0, 0, length, 0, 0, width, ColorRGBA::red);
      this->line(0, 0, 0, 0, length, 0, width, ColorRGBA::green);
      this->line(0, 0, 0, 0, 0, length, width, ColorRGBA::blue);
      break;
    }
    case Drawings3D::quad:
    {
      std::array<Vector3f, 4> points;
      ColorRGBA c;
      message.bin >> points[0];
      message.bin >> points[1];
      message.bin >> points[2];
      message.bin >> points[3];
      message.bin >> c;
      this->quad(points, c);
      break;
    }
    case Drawings3D::line:
    {
      Vector3f start, end;
      float width;
      ColorRGBA c;
      message.bin >> start;
      message.bin >> end;
      message.bin >> width;
      message.bin >> c;
      this->line(start, end, width, c);
      break;
    }
    case Drawings3D::cube:
    {
      Vector3f points[8];
      float width;
      ColorRGBA c;
      for(int i = 0; i < 8; i++)
        message.bin >> points[i];

      message.bin >> width;
      message.bin >> c;
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
      message.bin >> v;
      message.bin >> w;
      message.bin >> c;
      message.bin >> withLines;
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
      message.bin >> v;
      message.bin >> r;
      message.bin >> c;
      this->sphere(v, r, c);
      break;
    }
    case Drawings3D::ellipsoid:
    {
      Pose3f p;
      Vector3f s;
      ColorRGBA c;
      message.bin >> p;
      message.bin >> s;
      message.bin >> c;
      this->ellypsoid(p, s, c);
      break;
    }
    case Drawings3D::cylinder:
    {
      Vector3f v, rot;
      float r, r2, h;
      ColorRGBA c;
      message.bin >> v;
      message.bin >> rot;
      message.bin >> r;
      message.bin >> r2;
      message.bin >> h;
      message.bin >> c;
      this->cylinder(v, rot, r, r2, h, c);
      break;
    }
    case Drawings3D::partDisc:
    {
      Vector3f v,
               rot;
      float r, r2, a, a2;
      ColorRGBA c;
      message.bin >> v;
      message.bin >> rot;
      message.bin >> r;
      message.bin >> r2;
      message.bin >> a;
      message.bin >> a2;
      message.bin >> c;
      this->partDisc(v, rot, r, r2, a, a2, c);
      break;
    }
    case Drawings3D::image:
    {
      Vector3f v, rot;
      float w, h;
      CameraImage* ci = new CameraImage;
      message.bin >> v;
      message.bin >> rot;
      message.bin >> w;
      message.bin >> h;
      message.bin >> *ci;
      this->image(v, rot, w, h, ci);
      break;
    }
  }
  return true;
}

char* DebugDrawing3D::copyImage(const CameraImage& srcImage, unsigned int& width, unsigned int& height) const
{
  width = 1;
  while(width < srcImage.width)
    width <<= 1;
  height = 1;
  while(height < srcImage.height)
    height <<= 1;

  char* imageData = new char[width * height * 3];
  for(int y = srcImage.height - 1; y >= 0; y--)
  {
    unsigned char* p = reinterpret_cast<unsigned char*>(imageData + width * 3 * (srcImage.height - 1 - y));
    const CameraImage::PixelType* cur = &srcImage[y][0];
    const CameraImage::PixelType* end = cur + srcImage.width;
    for(; cur < end; cur++, p += 3)
    {
      ColorModelConversions::fromYUVToRGB(cur->y1, cur->u, cur->v, p[0], p[1], p[2]);
    }
  }
  return imageData;
}

DebugDrawing3D::Image3D::~Image3D()
{
  if(cameraImage)
    delete cameraImage;
}

const DebugDrawing3D::Image3D& DebugDrawing3D::Image3D::operator=(const DebugDrawing3D::Image3D& other)
{
  if(cameraImage)
    delete cameraImage;

  (Element&)*this = other;
  point = other.point;
  rotation = other.rotation;
  width = other.width;
  height = other.height;
  cameraImage = other.cameraImage;

  // dirty hack: works only for the current use of this class
  const_cast<Image3D&>(other).cameraImage = nullptr;
  return *this;
}
