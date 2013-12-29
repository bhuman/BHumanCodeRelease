/**
* @file Tools/Debugging/DebugDrawings3D.h
*/

#pragma once

#include "Tools/Debugging/DebugDrawings.h"

class Drawings3D
{
public:
  /** IDs for shape types.
  * Shapes are the basic drawings that can be sent.
  */
  enum ShapeType
  {
    dot,
    line,
    cube,
    polygon,
    quad,
    coordinates,
    scale,
    rotate,
    translate,
    sphere,
    cylinder,
    image
  };
};

/**
* singleton drawing manager class
*/
class DrawingManager3D : public DrawingManager {};

#ifdef RELEASE

#define DECLARE_DEBUG_DRAWING3D(id, type, ...) ((void) 0)

#else

/**
* A macro that declares
* @param id A drawing id
* @param type A coordinate-system type ("field", "robot", "camera" or "origin")
*/
#define DECLARE_DEBUG_DRAWING3D(id, type, ...) \
  Global::getDrawingManager3D().addDrawingId(id, type); \
  DEBUG_RESPONSE("debug drawing 3d:" id, __VA_ARGS__); \

#endif

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
* @param color The color of the line
*/
#define LINE3D(id, fromX, fromY, fromZ, toX, toY, toZ, size, color) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::line << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(fromX) << (float)(fromY) << (float)(fromZ) << (float)(toX) << (float)(toY) << (float)(toZ) << \
           (float)(size) << \
           ColorRGBA(color) \
          ); \
  })

/**
* A macro that adds a digit as a set of lines to a drawing.
* @param id The drawing to which the line should be added.
* @param digit The digit to draw.
* @param pos The coordinates of the top left corner.
* @param digitsize The size of the digit.
* @param linewidth The thickness of the line.
* @param color The color of the line
*/
#define DRAWDIGIT3D(id, digit, pos, digitsize, linewidth, color) do\
  {\
    static const Vector3<> points[8] =\
    {\
      Vector3<>(1, 0, 1),\
      Vector3<>(1, 0, 0),\
      Vector3<>(0, 0, 0),\
      Vector3<>(0, 0, 1),\
      Vector3<>(0, 0, 2),\
      Vector3<>(1, 0, 2),\
      Vector3<>(1, 0, 1),\
      Vector3<>(0, 0, 1)\
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
        Vector3<> from = pos - points[i] * digitsize;\
        Vector3<> to = pos - points[i + 1] * digitsize;\
        LINE3D(id, from.x, from.y, from.z, to.x, to.y, to.z, linewidth, color);\
      }\
  }\
  while(false)
/**
* A macro that adds a quadrilateral to a drawing.
* @param id The drawing to which the line should be added.
* @param corner1, corner2, corner3, corner4 The cornerpoints.
* @param color The color of the quadrilateral
*/
#define QUAD3D(id, corner1, corner2, corner3, corner4, color) \
  NOT_POLLABLE_DEBUG_RESPONSE( "debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::quad << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           Vector3<>(corner1) << Vector3<>(corner2) << Vector3<>(corner3) << Vector3<>(corner4) <<\
           ColorRGBA(color) \
          ); \
  })

/**
* A macro that adds a rectangle to a drawing.
* @param id The drawing to which the line should be added.
* @param a,b,c,d,e,f,g,h The x,y and z parameter of the cornerpoints.
* @param size The thickness of the line in pixels.
* @param color The color of the line
*/
#define CUBE3D(id, a, b, c, d, e, f, g, h, size, color) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::cube << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           Vector3<>(a) << Vector3<>(b) << Vector3<>(c) << Vector3<>(d) << Vector3<>(e) << Vector3<>(f) << \
           Vector3<>(g) << Vector3<>(h) <<\
           (float)(size) << \
           ColorRGBA(color) \
          ); \
  })

/**
* A macro that draws three lines on the three axis in positive direction.
* @param id The id of the drawing
* @param length The length of those lines in mm
* @param width The width of the lines in pixel
*/
#define COORDINATES3D(id, length, width) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::coordinates << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(length) << (float)(width) \
          ); \
  })

/**
* A macro that scales all drawing elements.
* Should be used to adjust the axes to those used be the robot
* (concerning -/+) (e.g. "robot",1.0f,-1.0f,1.0f)
* @param id The drawing the scaling should be applied to.
* @param x Scale factor for x axis.
* @param y Scale factor for y axis.
* @param z Scale factor for z axis.
*/
#define SCALE3D(id, x, y, z) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::scale << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) \
          ); \
  })

/**
* A macro that rotates the drawing around the three axes.
* First around x, then y, then z.
* @param id The drawing the rotation should be applied to.
* @param x Radiants to rotate ccw around the x axis.
* @param y Radiants to rotate ccw around the y axis.
* @param z Radiants to rotate ccw around the z axis.
*
*/
#define ROTATE3D(id, x, y, z) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::rotate << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) \
          ); \
  })

/**
* A macro that translates the drawing according to the given coordinates.
* The translation is the last thing done (after rotating and scaling)
* @param id The drawing the translation should be applied to.
* @param x mms to translate in x direction.
* @param y mms to translate in x direction.
* @param z mms to translate in x direction.
*
*/
#define TRANSLATE3D(id, x, y, z) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::translate << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) \
          ); \
  })

/** A macro that adds a point/dot to a drawing
* @param id The drawing to which the point will be added
* @param x The x-coordinate of the point.
* @param y The y-coordinate of the point.
* @param z The z-coordinate of the point.
* @param size The size of the point.
* @param color The color of the line
*/
#define POINT3D(id, x, y, z, size, color) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::dot << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) << \
           (float)size << \
           ColorRGBA(color) << false \
          ); \
  })

/** A macro that adds a sphere to a drawing
* @param id The drawing to which the sphere will be added
* @param x The x-coordinate of the sphere.
* @param y The y-coordinate of the sphere.
* @param z The z-coordinate of the sphere.
* @param radius The radius of the sphere.
* @param color The color of the line
*/
#define SPHERE3D(id, x, y, z, radius, color) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::sphere << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) << \
           (float) (radius) << \
           ColorRGBA(color) \
          ); \
  })

/** A macro that adds a cylinder to a drawing
* @param id The drawing to which the cylinder will be added
* @param x The x-coordinate of the cylinder.
* @param y The y-coordinate of the cylinder.
* @param z The z-coordinate of the cylinder.
* @param a The rotation around the x-axis in radians.
* @param b The rotation around the y-axis in radians.
* @param c The rotation around the z-axis in radians.
* @param radius The radius of the cylinder.
* @param height The height of the cylinder.
* @param color The color of the line
*/
#define CYLINDER3D(id, x, y, z, a, b, c, radius, height, color) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::cylinder << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) << \
           (float)(a) << (float)(b) << (float)(c) << \
           (float) (radius) << (float) (radius) << (float) (height) << \
           ColorRGBA(color) \
          ); \
  })

/**
* A macro that adds a cylinder to a drawing
* @param id The drawing to which the cylinder will be added
* @param x The x-coordinate of the cylinder.
* @param y The y-coordinate of the cylinder.
* @param z The z-coordinate of the cylinder.
* @param a The rotation around the x-axis in radians.
* @param b The rotation around the y-axis in radians.
* @param c The rotation around the z-axis in radians.
* @param baseRadius The radius at the base of the cylinder.
* @param topRadius The radius at the top of the cylinder.
* @param height The height of the cylinder.
* @param color The color of the line
*/
#define CYLINDER3D2(id, x, y, z, a, b, c, baseRadius, topRadius, height, color) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::cylinder << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) << \
           (float)(a) << (float)(b) << (float)(c) << \
           (float) (baseRadius) << (float) (topRadius) << (float) (height) << \
           ColorRGBA(color) \
          ); \
  })

/**
* A macro that adds an three-dimensional arrow to a drawing
* @param id The drawing to which the arrow will be added
* @param from The start point of the arrow
* @param to The end point of the arrow
* @param radius The radius of the arrow line.
* @param arrowlen The length of the arrowhead
* @param arrowradius The radius of the arrowhead
* @param color The color of the arrow
*/
#define CYLINDERARROW3D(id, from, to, radius, arrowlen, arrowradius, color) do \
  { \
    Vector3<> forward = to - from; \
    float height = forward.abs() - arrowlen; \
    forward.normalize(height * 0.5f); \
    Vector3<> center = from + forward; \
    float rx = 0.f, ry = 0.f; \
    rx = (forward.y != 0.f || forward.z != 0.f) ? -std::atan2(forward.y, forward.z) : 0.f; \
    float d = std::sqrt(forward.z * forward.z + forward.y * forward.y); \
    ry = (forward.x != 0.f || d != 0.f) ? std::atan2(forward.x, d) : 0.f; \
    CYLINDER3D(id, center.x, center.y, center.z, rx, ry, 0.f, radius, height, color); \
    forward.normalize(arrowlen * 0.5f); \
    Vector3<> arrowCenter = to - forward; \
    CYLINDER3D2(id, arrowCenter.x, arrowCenter.y, arrowCenter.z, rx, ry, 0.f, arrowradius, 0, arrowlen, color); \
  } \
  while(false)

/**
* A macro that adds a cylinder as a line to a drawing
* @param id The drawing to which the cylinder will be added
* @param from The start point of the line
* @param to The end point of the line
* @param radius The radius of the line.
* @param color The color of the line
*/
#define CYLINDERLINE3D(id, from, to, radius, color) do \
  { \
    Vector3<> forward = to - from; \
    float height = forward.abs(); \
    forward.normalize(height * 0.5f); \
    Vector3<> center = from + forward; \
    float rx = 0.f, ry = 0.f; \
    rx = (forward.y != 0.f || forward.z != 0.f) ? -std::atan2(forward.y, forward.z) : 0.f; \
    float d = std::sqrt(forward.z * forward.z + forward.y * forward.y); \
    ry = (forward.x != 0.f || d != 0.f) ? std::atan2(forward.x, d) : 0.f; \
    CYLINDER3D(id, center.x, center.y, center.z, rx, ry, 0.f, radius, height, color); \
  } \
  while(false)

/** A macro that adds an image to a drawing
* @param id The drawing to which the image will be added
* @param x The x-coordinate of the center of the image.
* @param y The y-coordinate of the center of the image.
* @param z The z-coordinate of the center of the image.
* @param a The rotation around the x-axis in radians.
* @param b The rotation around the y-axis in radians.
* @param c The rotation around the z-axis in radians.
* @param width The width of the image.
* @param height The height of the image.
* @param i The image.
*/
#define IMAGE3D(id, x, y, z, a, b, c, width, height, i) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, \
  { \
    OUTPUT(idDebugDrawing3D, bin, \
           (char)Drawings3D::image << \
           (char)Global::getDrawingManager3D().getDrawingId(id) << \
           (float)(x) << (float)(y) << (float)(z) << \
           (float)(a) << (float)(b) << (float)(c) << \
           (float) (width) << (float) (height) << \
           i \
          ); \
  })

/**
* Complex drawings should be encapsuled by this macro.
*/
#define COMPLEX_DRAWING3D(id, ...) \
  NOT_POLLABLE_DEBUG_RESPONSE("debug drawing 3d:" id, __VA_ARGS__; )
