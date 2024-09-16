/**
 * @file DebugDrawings3D.h
 */

#pragma once

#include "Debugging/DebugDrawings.h"
#include "Math/Eigen.h"
#include "RobotParts/FootShape.h"
#include <vector>

namespace Drawings3D
{
  /**
   * IDs for shape types.
   * Shapes are the basic drawings that can be sent.
   */
  enum ShapeType
  {
    dot,
    line,
    cube,
    quad,
    coordinates,
    scale,
    rotate,
    translate,
    sphere,
    ellipsoid,
    cylinder,
    image,
    renderOptions
  };

  /**
   * Bit masks for render options.
   * They can be passed to RENDER_OPTIONS3D.
   */
  enum RenderOption : unsigned char
  {
    disableTransparency = 1, /**< The drawing skips the pass with 50% transparency. */
    disableOpacity = 2, /**< The drawing skips the pass with 100% opacity. */
    disableDepth = 4, /**< The drawing will be overwritten by anything drawn afterwards. */
  };
};

class DrawingManager3D : public DrawingManager {};

#if !defined TARGET_ROBOT || !defined NDEBUG
/**
 * A macro that declares.
 * @param id A drawing id.
 * @param type A coordinate-system type ("field", "robot" or any
 *             robot part named in the scene description file).
 * and executes the following block if the drawing is requested.
 */
#define DEBUG_DRAWING3D(id, type) \
  if(Global::getDrawingManager3D().addDrawingId(id, type), _debugRequestActive("debug drawing 3d:" id))

/**
 * A macro that declares.
 * @param id A drawing id.
 * @param type A coordinate-system type ("field", "robot" or any
 *             robot part named in the scene description file).
 */
#define DECLARE_DEBUG_DRAWING3D(id, type) \
  do \
  { \
    Global::getDrawingManager3D().addDrawingId(id, type); \
    DECLARE_DEBUG_RESPONSE("debug drawing 3d:" id); \
  } \
  while(false)

/**
 * Complex drawings should be encapsulated by this macro.
 */
#define COMPLEX_DRAWING3D(id) \
  DECLARED_DEBUG_RESPONSE("debug drawing 3d:" id)

/**
 * A macro that adds a line to a drawing.
 * @param id The drawing to which the line should be added.
 * @param fromX The x-coordinate of the first point.
 * @param fromY The y-coordinate of the first point.
 * @param fromZ The z-coordinate of the first point.
 * @param toX The x-coordinate of the second point.
 * @param toY The y-coordinate of the second point.
 * @param toZ The z-coordinate of the second point.
 * @param size The thickness of the line in pixels.
 * @param color The color of the line.
 */
#define LINE3D(id, fromX, fromY, fromZ, toX, toY, toZ, size, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::line) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(fromX) << static_cast<float>(fromY) << static_cast<float>(fromZ) << \
             static_cast<float>(toX) << static_cast<float>(toY) << static_cast<float>(toZ) << \
             static_cast<float>(size) << \
             ColorRGBA(color) \
            ); \
    } \
  while(false)

/**
 * A macro that adds a cross to a drawing.
 * @param id The drawing to which the cross should be added.
 * @param x,y,z The center of the cross.
 * @param length stretching the cross by this amount.
 * @param size width of the cross lines.
 * @param penColor The color of the quadrangle.
 */
#define CROSS3D(id, x, y, z, length, size, penColor) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      LINE3D(id, x+length, y+length, z, x-length, y-length, z, size, penColor); \
      LINE3D(id, x+length, y-length, z, x-length, y+length, z, size, penColor); \
    } \
  while(false)

/**
 * A macro that adds a digit as a set of lines to a drawing.
 * @param id The drawing to which the line should be added.
 * @param digit The digit to draw.
 * @param pos The coordinates of the top left corner.
 * @param digitSize The size of the digit.
 * @param lineWidth The thickness of the line.
 * @param color The color of the line.
 */
#define DRAWDIGIT3D(id, digit, pos, digitSize, lineWidth, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    {\
      static const Vector3f points[8] =\
      {\
        Vector3f(1, 0, 1),\
        Vector3f(1, 0, 0),\
        Vector3f(0, 0, 0),\
        Vector3f(0, 0, 1),\
        Vector3f(0, 0, 2),\
        Vector3f(1, 0, 2),\
        Vector3f(1, 0, 1),\
        Vector3f(0, 0, 1)\
      };\
      static const unsigned char digits[10] =\
      {\
        0x3f,\
        0x0c,\
        0x76,\
        0x5e,\
        0x4d,\
        0x5b,\
        0x7b,\
        0x0e,\
        0x7f,\
        0x5f\
      };\
      digit = digits[std::abs(digit)];\
      for(int i = 0; i < 7; ++i)\
        if(digit & (1 << i))\
        {\
          Vector3f from = pos - points[i] * digitSize;\
          Vector3f to = pos - points[i + 1] * digitSize;\
          LINE3D(id, from.x(), from.y(), from.z(), to.x(), to.y(), to.z(), lineWidth, color);\
        }\
    }\
  while(false)

/**
 * A macro that adds a quadrilateral to a drawing.
 * @param id The drawing to which the line should be added.
 * @param corner1, corner2, corner3, corner4 The corner points.
 * @param color The color of the quadrilateral.
 */
#define QUAD3D(id, corner1, corner2, corner3, corner4, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::quad) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             Vector3f(corner1) << Vector3f(corner2) << Vector3f(corner3) << Vector3f(corner4) << \
             ColorRGBA(color) \
            ); \
    } \
  while(false)

/**
 * A macro that adds a rectangle to a drawing.
 * @param id The drawing to which the line should be added.
 * @param a,b,c,d,e,f,g,h The x,y and z parameter of the corner points.
 * @param size The thickness of the line in pixels.
 * @param color The color of the line.
 */
#define CUBE3D(id, a, b, c, d, e, f, g, h, size, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::cube) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             Vector3f(a) << Vector3f(b) << Vector3f(c) << Vector3f(d) << Vector3f(e) << Vector3f(f) << \
             Vector3f(g) << Vector3f(h) << \
             static_cast<float>(size) << \
             ColorRGBA(color) \
            ); \
    } \
  while(false)

/**
 * A macro that draws three lines on the three axis in positive direction.
 * @param id The id of the drawing.
 * @param length The length of those lines in mm.
 * @param width The width of the lines in pixel.
 */
#define COORDINATES3D(id, length, width) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::coordinates) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(length) << static_cast<float>(width) \
            ); \
    } \
  while(false)

#define SUBCOORDINATES3D(id, pose, length, width) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      const Pose3f& p = pose; \
      const Vector3f xAxis = p * Vector3f(length, 0, 0); \
      const Vector3f yAxis = p * Vector3f(0, length, 0); \
      const Vector3f zAxis = p * Vector3f(0, 0, length); \
      LINE3D(id, p.translation.x(), p.translation.y(), p.translation.z(), xAxis.x(), xAxis.y(), xAxis.z(), width, ColorRGBA::red); \
      LINE3D(id, p.translation.x(), p.translation.y(), p.translation.z(), yAxis.x(), yAxis.y(), yAxis.z(), width, ColorRGBA::green); \
      LINE3D(id, p.translation.x(), p.translation.y(), p.translation.z(), zAxis.x(), zAxis.y(), zAxis.z(), width, ColorRGBA::blue); \
    } \
  while(false)

/**
 * A macro that scales all drawing elements.
 * Should be used to adjust the axes to those used be the robot
 * (concerning -/+) (e.g. "robot",1.0f,-1.0f,1.0f).
 * @param id The drawing the scaling should be applied to.
 * @param x Scale factor for x axis.
 * @param y Scale factor for y axis.
 * @param z Scale factor for z axis.
 */
#define SCALE3D(id, x, y, z) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::scale) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) \
            ); \
    } \
  while(false)

/**
 * A macro that rotates the drawing around the three axes.
 * First around x, then y, then z.
 * @param id The drawing the rotation should be applied to.
 * @param x Radiants to rotate ccw around the x axis.
 * @param y Radiants to rotate ccw around the y axis.
 * @param z Radiants to rotate ccw around the z axis.
 */
#define ROTATE3D(id, x, y, z) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::rotate) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) \
            ); \
    } \
  while(false)

/**
 * A macro that translates the drawing according to the given coordinates.
 * The translation is the last thing done (after rotating and scaling).
 * @param id The drawing the translation should be applied to.
 * @param x mms to translate in x direction.
 * @param y mms to translate in y direction.
 * @param z mms to translate in z direction.
 */
#define TRANSLATE3D(id, x, y, z) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::translate) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) \
            ); \
    } \
  while(false)

/**
 * A macro that adds a point/dot to a drawing.
 * @param id The drawing to which the point will be added.
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 * @param z The z-coordinate of the point.
 * @param size The size of the point.
 * @param color The color of the line.
 */
#define POINT3D(id, x, y, z, size, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::dot) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) << \
             static_cast<float>(size) << \
             ColorRGBA(color) << false \
            ); \
    } \
  while(false)

/**
 * A macro that adds a sphere to a drawing.
 * @param id The drawing to which the sphere will be added.
 * @param x The x-coordinate of the sphere.
 * @param y The y-coordinate of the sphere.
 * @param z The z-coordinate of the sphere.
 * @param radius The radius of the sphere.
 * @param color The color of the line.
 */
#define SPHERE3D(id, x, y, z, radius, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::sphere) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) << \
             static_cast<float>(radius) << \
             ColorRGBA(color) \
            ); \
    } \
  while(false)

/**
 * A macro that adds an ellipsoid to a drawing.
 * @param id The drawing to which the sphere will be added.
 * @param p The pose of the ellipsoid as Pose3f.
 * @param r The radii of ellipsoid as Vector3f.
 * @param color The color of the ellipsoid.
 */
#define ELLIPSOID3D(id, p, r, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::ellipsoid) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             (p) << (r) << ColorRGBA(color)); \
    } \
  while(false)

/**
 * A macro that adds a cylinder to a drawing.
 * @param id The drawing to which the cylinder will be added.
 * @param x The x-coordinate of the cylinder.
 * @param y The y-coordinate of the cylinder.
 * @param z The z-coordinate of the cylinder.
 * @param a The rotation around the x-axis in radians.
 * @param b The rotation around the y-axis in radians.
 * @param c The rotation around the z-axis in radians.
 * @param radius The radius of the cylinder.
 * @param height The height of the cylinder.
 * @param color The color of the line.
 */
#define CYLINDER3D(id, x, y, z, a, b, c, radius, height, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::cylinder) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) << \
             static_cast<float>(a) << static_cast<float>(b) << static_cast<float>(c) << \
             static_cast<float>(radius) << static_cast<float>(radius) << static_cast<float>(height) << \
             ColorRGBA(color) \
            ); \
    } \
  while(false)

/**
 * A macro that adds a cylinder to a drawing.
 * @param id The drawing to which the cylinder will be added.
 * @param x The x-coordinate of the cylinder.
 * @param y The y-coordinate of the cylinder.
 * @param z The z-coordinate of the cylinder.
 * @param a The rotation around the x-axis in radians.
 * @param b The rotation around the y-axis in radians.
 * @param c The rotation around the z-axis in radians.
 * @param baseRadius The radius at the base of the cylinder.
 * @param topRadius The radius at the top of the cylinder.
 * @param height The height of the cylinder.
 * @param color The color of the line.
 */
#define CYLINDER3D2(id, x, y, z, a, b, c, baseRadius, topRadius, height, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::cylinder) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) << \
             static_cast<float>(a) << static_cast<float>(b) << static_cast<float>(c) << \
             static_cast<float>(baseRadius) << static_cast<float>(topRadius) << static_cast<float>(height) << \
             ColorRGBA(color) \
            ); \
    } \
  while(false)

/**
 * A macro that adds an three-dimensional arrow to a drawing.
 * @param id The drawing to which the arrow will be added.
 * @param from The start point of the arrow.
 * @param to The end point of the arrow.
 * @param radius The radius of the arrow line.
 * @param arrowLen The length of the arrowhead.
 * @param arrowRadius The radius of the arrowhead.
 * @param color The color of the arrow.
 */
#define CYLINDERARROW3D(id, from, to, radius, arrowLen, arrowRadius, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      Vector3f forward = (to) - (from); \
      float height = forward.norm() - (arrowLen); \
      forward.normalize(height * 0.5f); \
      Vector3f center = (from) + forward; \
      float rx = 0.f, ry = 0.f; \
      rx = (forward.y() != 0.f || forward.z() != 0.f) ? -std::atan2(forward.y(), forward.z()) : 0.f; \
      float d = std::sqrt(forward.z() * forward.z() + forward.y() * forward.y()); \
      ry = (forward.x() != 0.f || d != 0.f) ? std::atan2(forward.x(), d) : 0.f; \
      CYLINDER3D(id, center.x(), center.y(), center.z(), rx, ry, 0.f, radius, height, color); \
      forward.normalize((arrowLen) * 0.5f); \
      Vector3f arrowCenter = (to) - forward; \
      CYLINDER3D2(id, arrowCenter.x(), arrowCenter.y(), arrowCenter.z(), rx, ry, 0.f, arrowRadius, 0, arrowLen, color); \
    } \
  while(false)

/**
 * A macro that adds a cylinder as a line to a drawing.
 * @param id The drawing to which the cylinder will be added.
 * @param from The start point of the line.
 * @param to The end point of the line.
 * @param radius The radius of the line.
 * @param color The color of the line.
 */
#define CYLINDERLINE3D(id, from, to, radius, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      Vector3f forward = (to) - (from); \
      float height = forward.norm(); \
      forward.normalize(height * 0.5f); \
      Vector3f center = (from) + forward; \
      float rx = 0.f, ry = 0.f; \
      rx = (forward.y() != 0.f || forward.z() != 0.f) ? -std::atan2(forward.y(), forward.z()) : 0.f; \
      float d = std::sqrt(forward.z() * forward.z() + forward.y() * forward.y()); \
      ry = (forward.x() != 0.f || d != 0.f) ? std::atan2(forward.x(), d) : 0.f; \
      CYLINDER3D(id, center.x(), center.y(), center.z(), rx, ry, 0.f, radius, height, color); \
    } \
  while(false)

/**
 * A macro that adds a circle to a drawing.
 * @param id The drawing to which the circle will be added.
 * @param origin The pose of the circle center point
 *            Note: The circle will be created around the z-axis.
 * @param radius The radius of the circle.
 * @param wideness Defines the wideness of the circle.
 * @param color The color of the circle.
 */
#define CIRCLE3D(id, origin, radius, wideness, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      Vector3f from = (origin).translated(Vector3f(0, 0, -(wideness) / 2.f)).translation; \
      Vector3f to = (origin).translated(Vector3f(0, 0, (wideness) / 2.f)).translation; \
      CYLINDERLINE3D(id, from, to, radius, color); \
    } \
  while(false)

/**
 * A macro that adds a camera image to a drawing.
 * @param id The drawing to which the image will be added.
 * @param x The x-coordinate of the center of the image.
 * @param y The y-coordinate of the center of the image.
 * @param z The z-coordinate of the center of the image.
 * @param a The rotation around the x-axis in radians.
 * @param b The rotation around the y-axis in radians.
 * @param c The rotation around the z-axis in radians.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param ci The camera image.
 */
#define IMAGE3D(id, x, y, z, a, b, c, width, height, ci) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
             static_cast<char>(Drawings3D::image) << \
             Global::getDrawingManager3D().getDrawingId(id) << \
             static_cast<float>(x) << static_cast<float>(y) << static_cast<float>(z) << \
             static_cast<float>(a) << static_cast<float>(b) << static_cast<float>(c) << \
             static_cast<float>(width) << static_cast<float>(height) << \
             (ci) \
            ); \
    } \
  while(false)

/**
 * A macro that adds a foot to a drawing.
 * @param id The drawing to which the image will be added.
 * @param pose pose3f of the foot.
 * @param left true for a drawing of a left foot.
 * @param color The color of the foot.
 */
#define FOOT3D(id, pose, left, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      for(size_t i = 0; i < FootShape::polygon.size(); ++i) \
      { \
        Vector3f p1 = (Vector3f() << FootShape::polygon[i], 0.f).finished(); \
        Vector3f p2 = (Vector3f() << FootShape::polygon[(i + 1) % FootShape::polygon.size()], 0.f).finished(); \
        if(!(left)) \
        { \
          p1.y() = -p1.y(); \
          p2.y() = -p2.y(); \
        } \
        p1 = (pose) * p1; \
        p2 = (pose) * p2; \
        LINE3D(id, p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), 2, color); \
      } \
    } \
  while(false)

/**
 * Draw a horizontal ring sector.
 * @param id The drawing to which the ring sector will be added.
 * @param center The position of the center of the ring.
 * @param minAngle The smaller angle of the sector.
 * @param maxAngle The bigger angle of the sector. If it is smaller than minAngle,
 *                 2pi will be added automatically.
 * @param minRadius The inner radius of the ring.
 * @param maxRadius The outer radius of the ring.
 * @param color The color in which the sector is drawn.
 */
#define RING_SECTOR3D(id, center, minAngle, maxAngle, minRadius, maxRadius, color) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      const auto _quad = [](const Vector3f& _center, float _minAngle, float _maxAngle, float _minRadius, float _maxRadius, const ColorRGBA& _color) \
      { \
        const Vector3f _points[4] = \
        { \
          {_center.x() + std::cos(_minAngle) * _minRadius, _center.y() + std::sin(_minAngle) * _minRadius, _center.z()}, \
          {_center.x() + std::cos(_minAngle) * _maxRadius, _center.y() + std::sin(_minAngle) * _maxRadius, _center.z()}, \
          {_center.x() + std::cos(_maxAngle) * _maxRadius, _center.y() + std::sin(_maxAngle) * _maxRadius, _center.z()}, \
          {_center.x() + std::cos(_maxAngle) * _minRadius, _center.y() + std::sin(_maxAngle) * _minRadius, _center.z()} \
        }; \
        QUAD3D(id, _points[0], _points[1], _points[2], _points[3], _color); \
      }; \
      const Vector3f& _center = static_cast<const Vector3f&>(center); \
      const float _minAngle = static_cast<Angle>(minAngle); \
      float _maxAngle = static_cast<float>(maxAngle); \
      const float _minRadius = static_cast<float>(minRadius); \
      const float _maxRadius = static_cast<float>(maxRadius); \
      const ColorRGBA& _color = static_cast<const ColorRGBA&>(color); \
      if(_maxAngle < _minAngle) \
        _maxAngle += pi2; \
      const float _step = pi2 / 32.f; \
      const float _minFull = std::ceil(_minAngle / _step) * _step; \
      const float _maxFull = std::floor(_maxAngle / _step) * _step; \
      if(_maxFull <= _minFull + 0.0001f) \
        _quad(_center, _minAngle, _maxAngle, _minRadius, _maxRadius, _color); \
      else \
      { \
        if(_minAngle < _minFull - 0.0001f) \
          _quad(_center, _minAngle, _minFull, _minRadius, _maxRadius, _color); \
        float _angle = _minFull; \
        for(; _angle <= _maxFull - _step; _angle += _step) \
          _quad(_center, _angle, _angle + _step, _minRadius, _maxRadius, _color); \
        if(_angle < _maxAngle - 0.0001f) \
          _quad(_center, _angle, _maxAngle, _minRadius, _maxRadius, _color); \
      } \
    } \
  while(false)

/**
 * Specify specific rendering options for a drawing.
 * @param id The drawing in which the options will be set.
 * @param options The bits for the different options. Currently, only
 *                Drawings3D::disableTransparency is supported.
 */
#define RENDER_OPTIONS3D(id, options) \
  do \
    COMPLEX_DRAWING3D(id) \
    { \
      OUTPUT(idDebugDrawing3D, bin, \
        static_cast<char>(Drawings3D::renderOptions) << \
        Global::getDrawingManager3D().getDrawingId(id) << \
        static_cast<char>(options)); \
    } \
  while(false)

#else
//Ignore everything
#define DEBUG_DRAWING3D(id, type) if(false)
#define DECLARE_DEBUG_DRAWING3D(id, type) static_cast<void>(0)
#define COMPLEX_DRAWING3D(id) if(false)
#define SPHERE3D(id, x, y, z, radius, color) static_cast<void>(0)
#define CIRCLE3D(id, origin, radius, wideness, color) static_cast<void>(0)
#define CYLINDERLINE3D(id, from, to, radius, color) static_cast<void>(0)
#define POINT3D(id, x, y, z, size, color) static_cast<void>(0)
#define LINE3D(id, fromX, fromY, fromZ, toX, toY, toZ, size, color) static_cast<void>(0)
#define CROSS3D(id, x, y, z, length, size, penColor) static_cast<void>(0)
#define DRAWDIGIT3D(id, digit, pos, digitSize, lineWidth, color) static_cast<void>(0)
#define QUAD3D(id, corner1, corner2, corner3, corner4, color) static_cast<void>(0)
#define CUBE3D(id, a, b, c, d, e, f, g, h, size, color) static_cast<void>(0)
#define COORDINATES3D(id, length, width) static_cast<void>(0)
#define SUBCOORDINATES3D(id, pose, length, width) static_cast<void>(0)
#define SCALE3D(id, x, y, z) static_cast<void>(0)
#define ROTATE3D(id, x, y, z) static_cast<void>(0)
#define TRANSLATE3D(id, x, y, z) static_cast<void>(0)
#define ELLIPSOID3D(id, p, r, color) static_cast<void>(0)
#define CYLINDER3D(id, x, y, z, a, b, c, radius, height, color) static_cast<void>(0)
#define CYLINDER3D2(id, x, y, z, a, b, c, baseRadius, topRadius, height, color) static_cast<void>(0)
#define CYLINDERARROW3D(id, from, to, radius, arrowLen, arrowRadius, color) static_cast<void>(0)
#define IMAGE3D(id, x, y, z, a, b, c, width, height, i) static_cast<void>(0)
#define FOOT3D(id, pose, left, color) static_cast<void>(0)
#define RING_SECTOR3D(id, center, minAngle, maxAngle, minRadius, maxRadius, color) static_cast<void>(0)
#define RENDER_OPTIONS3D(id, options) static_cast<void>(0)
#endif
