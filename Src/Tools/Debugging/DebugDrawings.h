/**
 * @file Tools/Debugging/DebugDrawings.h
 */

#pragma once

#include <unordered_map>

#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Eigen.h"

namespace Drawings
{
  /**
   * IDs for shape types
   * shapes are the basic drawings that can be sent.
   */
  enum ShapeType
  {
    arc, arrow, circle, dot, dotLarge, dotMedium, ellipse,
    line, origin, polygon, rectangle, text, tip, robot, spot
  };

  /** The pen style that is used for basic shapes*/
  enum PenStyle
  {
    noPen, solidPen, dashedPen, dottedPen
  };

  /** The brush style that is used for basic shapes*/
  enum BrushStyle
  {
    noBrush, solidBrush
  };
};

/**
 * Singleton drawing manager class
 */
class DrawingManager
{
public:
  class Drawing
  {
  public:
    char id;
    char type;
    char processIdentifier;
  };

  std::unordered_map<const char*, Drawing> drawings;

private:
  std::unordered_map<std::string, const char*> strings;
  std::unordered_map<const char*, char> types;
  char processIdentifier = 0;

  std::unordered_map<unsigned int, const char*> drawingsById;
  std::unordered_map<unsigned int, const char*> typesById;

  // only a process is allowed to create the instance.
  friend class Process;
  friend class RobotConsole;
  friend class DrawingManager3D;
  friend In& operator>>(In& stream, DrawingManager&);
  friend Out& operator<<(Out& stream, const DrawingManager&);

private:
  /**
   * No other instance of this class is allowed except the one accessible via getDrawingManager
   * therefore the constructor is private.
   */
  DrawingManager() = default;
  DrawingManager(const DrawingManager&) = delete;

public:
  void clear();
  void addDrawingId(const char* name, const char* typeName);
  char getDrawingId(const char* name) const;
  const char* getDrawingType(const char* name) const;
  const char* getDrawingName(char id) const;
  const char* getString(const std::string& string);
  void setProcess(char processIdentifier)
  {
    this->processIdentifier = processIdentifier;
  }

private:
  const char* getTypeName(char id, char processIdentifier) const;
};

In& operator>>(In& stream, DrawingManager&);
Out& operator<<(Out& stream, const DrawingManager&);

inline char DrawingManager::getDrawingId(const char* name) const
{
  std::unordered_map<const char*, Drawing>::const_iterator i = drawings.find(name);
  if(i != drawings.end())
    return i->second.id;
  OUTPUT_WARNING("Debug drawing " << name << " not declared");
  return -1;
}

inline const char* DrawingManager::getDrawingType(const char* name) const
{
  std::unordered_map< const char*, Drawing>::const_iterator i = drawings.find(name);
  if(i != drawings.end())
    return getTypeName(i->second.type, i->second.processIdentifier);
  OUTPUT_WARNING("Debug drawing " << name << " not declared");
  return "unknown";
}

inline const char* DrawingManager::getDrawingName(char id) const
{
  unsigned int key = static_cast<unsigned int>(processIdentifier) << 24 | static_cast<unsigned int>(id);
  std::unordered_map<unsigned int, const char*>::const_iterator i = drawingsById.find(key);
  if(i != drawingsById.end())
    return i->second;
  OUTPUT_WARNING("Unknown debug drawing id " << int(id));
  return "unknown";
}

inline const char* DrawingManager::getTypeName(char id, char processIdentifier) const
{
  unsigned int key = static_cast<unsigned int>(processIdentifier) << 24 | static_cast<unsigned int>(id);
  std::unordered_map<unsigned int, const char*>::const_iterator i = typesById.find(key);
  if(i != typesById.end())
    return i->second;
  OUTPUT_WARNING("Debug drawing has unknown type " << int(id));
  return "unknown";
}

#if !defined TARGET_TOOL && (!defined TARGET_ROBOT || !defined NDEBUG)

/**
 * A macro that declares
 * @param id A drawing id
 * @param type A drawing type
 * and executes the following block if the drawing is requested.
 */
#define DEBUG_DRAWING(id, type) \
  if(Global::getDrawingManager().addDrawingId(id, type), _debugRequestActive("debug drawing:" id))

/**
 * A macro that declares
 * @param id A drawing id
 * @param type A drawing type
 */
#define DECLARE_DEBUG_DRAWING(id, type) \
  do \
  { \
    Global::getDrawingManager().addDrawingId(id, type); \
    DECLARE_DEBUG_RESPONSE("debug drawing:" id); \
  } \
  while(false)

/**
 * Complex drawings should be encapsuled by this macro.
 * @param id A drawing id
 */
#define COMPLEX_DRAWING(id) \
  DECLARED_DEBUG_RESPONSE("debug drawing:" id)

/**
 * A macro that sends a circle
 * @param id A drawing id
 * @param center_x The x coordinate of the center of the circle
 * @param center_y The y coordinate of the center of the circle
 * @param radius The radius of the circle
 * @param penWidth The width of the arc of the circle
 * @param penStyle The pen style of the arc of the circle (Drawings::PenStyle)
 * @param penColor The color of the arc of the circle
 * @param brushStyle The brush style of the circle
 * @param brushColor The brush color of the circle
 */
#define CIRCLE(id, center_x, center_y, radius, penWidth, penStyle, penColor, brushStyle, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::circle << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(center_x) << (int)(center_y) << (int)(radius) << (char)(penWidth) << \
             (char)(penStyle) << ColorRGBA(penColor) << (char)(brushStyle) << ColorRGBA(brushColor)\
            ); \
    } \
  while(false)

/**
 * A macro that sends an arc
 * @param id A drawing id
 * @param center_x The x coordinate of the center of the arc
 * @param center_y The y coordinate of the center of the arc
 * @param radius The radius of the arc
 * @param startAngle The starting angle of the arc
 * @param spanAngle The spanning angle of the arc. The endAngle is startAngle + spanAngle.
 * @param penWidth The width of the arc
 * @param penStyle The pen style of the arc (Drawings::PenStyle)
 * @param penColor The color of the arc
 * @param brushStyle The brush style of the arc
 * @param brushColor The brush color of the arc
 */
#define ARC(id, center_x, center_y, radius, startAngle, spanAngle, penWidth, penStyle, penColor, brushStyle, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::arc << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(center_x) << (int)(center_y) << (int)(radius) << \
             Angle(startAngle) << Angle(spanAngle) << \
             (char)(penWidth) << \
             (char)(penStyle) << ColorRGBA(penColor) << (char)(brushStyle) << ColorRGBA(brushColor)\
            ); \
    } \
  while(false)

/**
 * A macro that sends an ellipse
 * @param id A drawing id
 * @param center The coordinate of the center of the ellipse (Vector2f)
 * @param radiusX The radius in x direction
 * @param radiusY The radius in y direction
 * @param rotation The rotation of the x axis
 * @param penWidth The width of the arc of the ellipse
 * @param penStyle The pen style of the arc of the ellipse (Drawings::PenStyle)
 * @param penColor The color of the arc of the ellipse
 * @param brushStyle The brush style of the ellipse
 * @param brushColor The brush color of the ellipse
 */
#define ELLIPSE(id, center, radiusX, radiusY, rotation, penWidth, penStyle, penColor, brushStyle, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::ellipse << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(center).x() << (int)(center).y() << (int)(radiusX) << (int)(radiusY) << (float)(rotation) << (char)(penWidth) << \
             (char)(penStyle) << ColorRGBA(penColor) << (char)(brushStyle) << ColorRGBA(brushColor)\
            ); \
    } \
  while(false)

/**
 * A macro that sends a Rectangle
 * @param id A drawing id
 * @param topLeft The coordinate of the top left corner of the rectangle without rotation (Vector2f)
 * @param rotation The rotation of the x axis
 * @param penWidth The width of the border
 * @param penStyle The pen style of the border (Drawings::PenStyle)
 * @param penColor The color of the border
 * @param brushStyle The brush style of the rectangle
 * @param brushColor The brush color of the rectangle
 */
#define RECTANGLE2(id, topLeft, width, height, rotation, penWidth, penStyle, penColor, brushStyle, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::rectangle << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(topLeft).x() << (int)(topLeft).y() << (int)(width) << (int)(height) << (float) (rotation) << (char)(penWidth) << \
             (char)(penStyle) << ColorRGBA(penColor) << (char)(brushStyle) << ColorRGBA(brushColor)\
            ); \
    } \
  while(false)

/**
 * A macro that sends a polygon
 * @param id A drawing id (Drawings::FieldDrawing or Drawings::ImageDrawing)
 * @param numberOfPoints The number the points of the polygon
 * @param points A list which contains the points of the polygon
 * @param penWidth The width of the pen
 * @param penStyle The pen style of the arc of the circle (Drawings::PenStyle)
 * @param penColor The color of the arc of the circle
 * @param brushStyle The brush style of the polygon
 * @param brushColor The brush color of the polygon
 */
#define POLYGON(id, numberOfPoints, points, penWidth, penStyle, penColor, brushStyle, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OutTextMemory _stream((int)(numberOfPoints) * 12); \
      for(int _i = 0; _i < (int)(numberOfPoints); ++_i) \
        _stream << (int)(points)[_i].x() << (int)(points)[_i].y(); \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::polygon << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(numberOfPoints) << \
             _stream.data() << \
             (char)(penWidth) << (char)(penStyle) << ColorRGBA(penColor) << \
             (char)(brushStyle) << ColorRGBA(brushColor) \
            );  \
    } \
  while(false)

/**
 * A macro that sends a dot (a quadratic box with a border)
 * @param id A drawing id
 * @param x The x coordinate of the center of the box
 * @param y The y coordinate of the center of the box
 * @param penColor The color of the border of the dot (Drawings::Color)
 * @param brushColor The color of the dot (Drawings::Color)
 */
#define DOT(id, x, y, penColor, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::dot << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << ColorRGBA(penColor) << ColorRGBA(brushColor) \
            ); \
    } \
  while(false)

/**
 * A macro that sends a dot (a quadratic box with a border)
 * @param id A drawing id
 * @param xy The coordinates of the center of the box
 * @param penColor The color of the border of the dot (Drawings::Color)
 * @param brushColor The color of the dot (Drawings::Color)
 */
#define DOT_AS_VECTOR(id, xy, penColor, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::dot << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(xy).x() << (int)(xy).y() << ColorRGBA(penColor) << ColorRGBA(brushColor) \
            ); \
    } \
  while(false)

/**
 * A macro that sends a dot (a quadratic box with a border)
 * @param id A drawing id
 * @param x The x coordinate of the center of the box
 * @param y The y coordinate of the center of the box
 * @param penColor The color of the border of the dot (Drawings::Color)
 * @param brushColor The color of the dot (Drawings::Color)
 */
#define MID_DOT(id, x, y, penColor, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::dotMedium << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << ColorRGBA(penColor) << ColorRGBA(brushColor) \
            ); \
    } \
  while(false)

/**
 * A macro that sends a dot (a quadratic box with a border)
 * @param id A drawing id
 * @param x The x coordinate of the center of the box
 * @param y The y coordinate of the center of the box
 * @param penColor The color of the border of the dot (Drawings::Color)
 * @param brushColor The color of the dot (Drawings::Color)
 */
#define LARGE_DOT(id, x, y, penColor, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::dotLarge << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << ColorRGBA(penColor) << ColorRGBA(brushColor) \
            ); \
    } \
  while(false)

/**
 * A macro that sends a line with fload values
 * @param id A drawing id
 * @param x1 The x coordinate of the starting point.
 * @param y1 The y coordinate of the starting point.
 * @param x2 The x coordinate of the end point.
 * @param y2 The y coordinate of the end point.
 * @param penWidth The width of the line
 * @param penStyle The pen style of the line (Drawings::PenStyle)
 * @param penColor The color of the line (Drawings::Color)
 */
#define LINE(id, x1, y1, x2, y2, penWidth, penStyle, penColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::line << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (float)(x1) << (float)(y1) << (float)(x2) << (float)(y2) << (float)(penWidth) << (char)(penStyle) << ColorRGBA(penColor) \
            ); \
    } \
  while(false)

#define RAY(id, base, angle, penWidth, penStyle, penColor) \
  COMPLEX_DRAWING(id) \
  { \
    const Vector2f to = (base) + Vector2f(10000.f,0.f).rotate(angle); \
    LINE(id, (base).x(), (base).y(), to.x(), to.y(), penWidth, penStyle, penColor); \
  }

/**
 * A macro that sends an arrow
 * @param id A drawing id
 * @param x1 The x coordinate of the starting point.
 * @param y1 The y coordinate of the starting point.
 * @param x2 The x coordinate of the end point.
 * @param y2 The y coordinate of the end point.
 * @param penWidth The width of the line
 * @param penStyle The pen style of the line (Drawings::PenStyle)
 * @param penColor The color of the line (Drawings::Color)
 */
#define ARROW(id, x1, y1, x2, y2, penWidth, penStyle, penColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::arrow << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (float)(x1) << (float)(y1) << (float)(x2) << (float)(y2) << (float)(penWidth) << (char)(penStyle) << ColorRGBA(penColor) \
            ); \
    } \
  while(false)

/**
 * A macro that sends an quadrangle
 * @param x1,y1,x2,y2,x3,y3,x4,y4 The coordinates of the 4 quadrangle vertices
 * @param id A drawing id
 * @param penWidth The line width of the quadrangle
 * @param penStyle The line style, e.g. dotted
 * @param penColor The color of the quadrangle
 */
#define QUADRANGLE(id, x1, y1, x2, y2, x3, y3, x4, y4, penWidth, penStyle, penColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      LINE(id, x1, y1, x2, y2, penWidth, penStyle, penColor); \
      LINE(id, x2, y2, x3, y3, penWidth, penStyle, penColor); \
      LINE(id, x3, y3, x4, y4, penWidth, penStyle, penColor); \
      LINE(id, x4, y4, x1, y1, penWidth, penStyle, penColor); \
    } \
  while(false)

/**
 * A macro that sends an rectangle
 * @param x1,y1,x2,y2 The coordinates of 2 opposite corners
 * @param id A drawing id
 * @param penWidth The line width of the rectangle
 * @param penStyle The line style, e.g. dotted
 * @param penColor The color of the quadrangle
 */
#define RECTANGLE(id, x1, y1, x2, y2, penWidth, penStyle, penColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      LINE(id, x1, y1, x1, y2, penWidth, penStyle, penColor); \
      LINE(id, x1, y2, x2, y2, penWidth, penStyle, penColor); \
      LINE(id, x2, y2, x2, y1, penWidth, penStyle, penColor); \
      LINE(id, x2, y1, x1, y1, penWidth, penStyle, penColor); \
    } \
  while(false)

/**
 * A macro that sends a filled rectangle
 * @param x1,y1,x2,y2 The coordinates of 2 opposite corners
 * @param id A drawing id
 * @param penWidth The line width of the rectangle
 * @param penStyle The line style, e.g. dotted
 * @param penColor The color of the quadrangle
 * @param brushStyle The brush style of the polygon
 * @param brushColor The brush color of the polygon
 */
#define FILLED_RECTANGLE(id, x1, y1, x2, y2, penWidth, penStyle, penColor, brushStyle, brushColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      Vector2i points[4]; \
      points[0] = Vector2i(int(x1), int(y1)); \
      points[1] = Vector2i(int(x2), int(y1)); \
      points[2] = Vector2i(int(x2), int(y2)); \
      points[3] = Vector2i(int(x1), int(y2)); \
      POLYGON(id, 4, points, penWidth, penStyle, penColor, brushStyle, brushColor); \
    } \
  while(false)

/**
 * A macro that sends a cross
 * @param x,y The center of the cross
 * @param size Half of the height of the rectangle enclosing the cross
 * @param id A drawing id
 * @param penWidth The line width of the rectangle
 * @param penStyle The line style, e.g. dotted
 * @param penColor The color of the quadrangle
 */
#define CROSS(id, x, y, size, penWidth, penStyle, penColor) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      LINE(id, (x)+size, (y)+size, (x)-size, (y)-size, penWidth, penStyle, penColor); \
      LINE(id, (x)+size, (y)-size, (x)-size, (y)+size, penWidth, penStyle, penColor); \
    } \
  while(false)

/**
 * A macro that sends a text
 * @param id A drawing id
 * @param x The x coordinate of the upper left corner of the text
 * @param y The y coordinate of the upper left corner of the text
 * @param fontSize The size of the font of the text
 * @param color The color of the text
 * @param txt The text (streaming is possible)
 */
#define DRAWTEXT(id, x, y, fontSize, color, txt) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OutTextRawMemory _stream; \
      _stream << txt; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::text << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << (short)(fontSize) << (ColorRGBA)(color) << _stream.data() \
            ); \
    } \
  while(false)

/** A macro that sends a spot with clickable action
 * TODO
 */
#define SPOT(id, x1, y1, x2, y2, action) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OutTextRawMemory _stream(1024); \
      _stream << action; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::spot << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x1) << (int)(y1) << (int)(x2) << int(y2) << _stream.data() \
            ); \
    } \
  while(false)

/**
 * A macro that sends a tip (popup text)
 * @param id A drawing id
 * @param x The x coordinate of the center of the anchor area
 * @param y The y coordinate of the center of the anchor area
 * @param radius The radius of the anchor area
 * @param text The text (streaming is possible)
 */
#define TIP(id, x, y, radius, text) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OutTextRawMemory _stream(1024); \
      _stream << text; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::tip << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << (int)(radius) << _stream.data() \
            ); \
    } \
  while(false)

/**
 * A macro that defines a new origin
 * @param id A drawing id
 * @param x The x coordinate of the new origin.
 * @param y The y coordinate of the new origin.
 * @param angle The orientation of the new origin.
 */
#define ORIGIN(id, x, y, angle) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::origin << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << (float)(angle) \
            ); \
    } \
  while(false)

/**
 * A macro that sends a RobotPose (Pose2f)
 * @param id A drawing id
 * @param p The desired Pose2f
 * @param dirVec The direction vector of the body
 * @param dirHeadVec The direction vector of the head
 * @param alphaRobot The alpha of the robot to draw
 * @param colorBody The color of the robot
 * @param colorDirVec The color of the direction vector of the body (Set alpha channel to 0, to disable this drawing)
 * @param colorDirHeadVec The color of the direction vector of the head (Set alpha channel to 0, to disable this drawing)
 */
#define ROBOT(id, p, dirVec, dirHeadVec, alphaRobot, colorBody, colorDirVec, colorDirHeadVec) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::robot << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             Pose2f(p) << Vector2f(dirVec) << Vector2f(dirHeadVec) << \
             (float)(alphaRobot) << ColorRGBA(colorBody) << ColorRGBA(colorDirVec) << ColorRGBA(colorDirHeadVec)); \
    } \
  while(false)

/**
 * A macro that sends a RobotPose (Pose2f)
 * @param id A drawing id
 * @param p The desired Pose2f
 * @param color The desired color for this drawing
 */
#define DRAW_ROBOT_POSE(id, p, color) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      Vector2f dirVec(200.f, 0.f); \
      dirVec.rotate((p).rotation); \
      dirVec += (p).translation; \
      ROBOT(id, p, dirVec, dirVec, 1.f, color, ColorRGBA::white, ColorRGBA(0, 0, 0, 0)); \
    } \
  while(false)

/**
 * A macro that sends a RobotPose (Pose2f)
 * @param id A drawing id
 * @param p The desired Pose2f
 * @param color The desired color for this drawing
 * @param headRotation the Z-rotation of the head
 */
#define DRAW_ROBOT_POSE_WITH_HEAD_ROTATION(id, p, color, headRotation) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      Vector2f dirVec(200.f, 0.f); \
      dirVec.rotate((p).rotation); \
      dirVec += (p).translation; \
      Vector2f dirHeadVec(150.f, 0.f); \
      dirHeadVec.rotate((p).rotation); \
      dirHeadVec.rotate(headRotation); \
      dirHeadVec += (p).translation; \
      ROBOT(id, p, dirVec, dirHeadVec, 1.f, color, ColorRGBA::white, ColorRGBA::violet); \
    } \
  while(false)

/**
 * A macro that sends a RobotPose (Pose2f) and draws and arc for the rotational standard deviation
 * @param id A drawing id
 * @param p The desired Pose2f
 * @param stdDev The standard deviation
 * @param color The desired color for this drawing
 */
#define DRAW_ROBOT_POSE_ROTATIONAL_STANDARD_DEVIATION(id, p, stdDev, color) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      DRAW_ROBOT_POSE(id, p, color); \
      ARC(id, (p).translation.x(), (p).translation.y(), \
          200.f, (-(stdDev)/2.f + (p).rotation), stdDev, 20, Drawings::solidPen, ColorRGBA::white, \
          Drawings::noBrush, ColorRGBA::white); \
    } \
  while(false)

/**
 * A macro that send a covariance ellipses (overlayed) with default confidence intervals
 * @param id A drawing id
 * @param cov The covariance matrix as Matrix2f
 * @param mean The mean value
 */
#define COVARIANCE_ELLIPSES_2D(id, cov, mean) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      float axis1; \
      float axis2; \
      float angle; \
      Covariance::errorEllipse(cov, axis1, axis2, angle, 0.99f); \
      ELLIPSE(id, mean, axis1/2.f, axis2/2.f, angle, \
              10, Drawings::solidPen, ColorRGBA(100,100,255,100), Drawings::solidBrush, ColorRGBA(100,100,255,100)); \
      Covariance::errorEllipse(cov, axis1, axis2, angle, 0.95f); \
      ELLIPSE(id, mean, axis1/2.f, axis2/2.f, angle, \
              10, Drawings::solidPen, ColorRGBA(150,150,100,100), Drawings::solidBrush, ColorRGBA(150,150,100,100)); \
      Covariance::errorEllipse(cov, axis1, axis2, angle, 0.68f); \
      ELLIPSE(id, mean, axis1/2.f, axis2/2.f, angle, \
              10, Drawings::solidPen, ColorRGBA(255,100,100,100), Drawings::solidBrush, ColorRGBA(255,100,100,100)); \
    } \
  while(false)

/**
 * A macro that send a single covariance ellipse of arbitrary color and confidence interval
 * @param id A drawing id
 * @param cov The covariance matrix as Matrix2f
 * @param mean The mean value
 * @param p The confidence interval
 * @param color A color definition
 */
#define COVARIANCE_ELLIPSE_2D(id, cov, mean, p, color) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      float axis1; \
      float axis2; \
      float angle; \
      Covariance::errorEllipse(cov, axis1, axis2, angle, p); \
      ELLIPSE(id, mean, axis1/2.f, axis2/2.f, angle, 10, Drawings::solidPen, color, Drawings::solidBrush, color); \
    } \
  while(false)

/**
 * A macro that plots a value.
 * These values are collected and plotted over time.
 * @param id The name of the plot.
 * @param value The value to be plotted.
 */
#define PLOT(id, value) \
  do \
    DEBUG_RESPONSE("plot:" id) OUTPUT(idPlot, bin, id << (float)(value)); \
  while(false)

/**
 * A macro that declares a pollable plot.
 * @param id The name of the plot.
 */
#define DECLARE_PLOT(id) \
  DECLARE_DEBUG_RESPONSE("plot:" id)

/**
 * A macro that creates three plots for the three axis of a vector3
 */
#define DECLARE_VEC3_PLOT(id) \
  do \
  { \
    DECLARE_PLOT(id "X"); \
    DECLARE_PLOT(id "Y"); \
    DECLARE_PLOT(id "Z"); \
  } \
  while(false)

/**
 * A macro that plots the three values of a vector3.
 */
#define PLOT_VEC3(id, vec) \
  do \
  { \
    PLOT(id "X", vec.x()); \
    PLOT(id "Y", vec.y()); \
    PLOT(id "Z", vec.z()); \
  } \
  while(false)

#else
//Ignore everything
#define DEBUG_DRAWING(id, type) if(false)
#define DECLARE_DEBUG_DRAWING(id, type) ((void) 0)
#define COMPLEX_DRAWING(id) if(false)
#define CIRCLE(id, center_x, center_y, radius, penWidth, penStyle, penColor, brushStyle, brushColor) ((void) 0)
#define ARC(id, center_x, center_y, radius, startAngle, spanAngle, penWidth, penStyle, penColor, brushStyle, brushColor) ((void) 0)
#define ELLIPSE(id, center, radiusX, radiusY, rotation, penWidth, penStyle, penColor, brushStyle, brushColor) ((void) 0)
#define RECTANGLE2(id, topLeft, width, height, rotation, penWidth, penStyle, penColor, brushStyle, brushColor) ((void) 0)
#define POLYGON(id, numberOfPoints, points, penWidth, penStyle, penColor, brushStyle, brushColor) ((void) 0)
#define DOT(id, x, y, penColor, brushColor) ((void) 0)
#define DOT_AS_VECTOR(id, xy, penColor, brushColor) ((void) 0)
#define MID_DOT(id, x, y, penColor, brushColor) ((void) 0)
#define LARGE_DOT(id, x, y, penColor, brushColor) ((void) 0)
#define LINE(id, x1, y1, x2, y2, penWidth, penStyle, penColor) ((void) 0)
#define RAY(id, base, angle, penWidth, penStyle, penColor) ((void) 0)
#define ARROW(id, x1, y1, x2, y2, penWidth, penStyle, penColor) ((void) 0)
#define QUADRANGLE(id, x1, y1, x2, y2, x3, y3, x4, y4, penWidth, penStyle, penColor) ((void) 0)
#define RECTANGLE(id, x1, y1, x2, y2, penWidth, penStyle, penColor) ((void) 0)
#define FILLED_RECTANGLE(id, x1, y1, x2, y2, penWidth, penStyle, penColor, brushStyle, brushColor) ((void) 0)
#define CROSS(id, x, y, size, penWidth, penStyle, penColor) ((void) 0)
#define DRAWTEXT(id, x, y, fontSize, color, txt) ((void) 0)
#define SPOT(id, x1, y1, x2, y2, action) ((void) 0)
#define TIP(id, x, y, radius, text) ((void) 0)
#define ORIGIN(id, x, y, angle) ((void) 0)
#define ROBOT(id, p, dirVec, dirHeadVec, alphaRobot, colorBody, colorDirVec, colorDirHeadVec) ((void) 0)
#define DRAW_ROBOT_POSE(id, p, color) ((void) 0)
#define DRAW_ROBOT_POSE_WITH_HEAD_ROTATION(id, p, color, headRotation) ((void) 0)
#define DRAW_ROBOT_POSE_ROTATIONAL_STANDARD_DEVIATION(id, p, stdDev, color) ((void) 0)
#define COVARIANCE_ELLIPSES_2D(id, cov, mean) ((void) 0)
#define COVARIANCE_ELLIPSE_2D(id, cov, mean, p, color) ((void) 0)
#define PLOT(id, value) ((void) 0)
#define DECLARE_PLOT(id) ((void) 0)
#define DECLARE_VEC3_PLOT(id) ((void) 0)
#define PLOT_VEC3(id, vec) ((void) 0)
#endif
