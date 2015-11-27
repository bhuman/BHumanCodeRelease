/**
 * @file Tools/Debugging/DebugDrawings.h
 */

#pragma once

#include <unordered_map>

#include "Tools/ColorRGBA.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"

namespace Drawings
{
  /**
   * IDs for shape types
   * shapes are the basic drawings that can be sent.
   */
  enum ShapeType
  {
    circle, polygon, ellipse, line, dot, midDot, largeDot,
    arrow, text, tip, origin, gridRGBA, gridMono, rectangle,
    arc
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

class Framework;

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
  friend class Framework;
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
  void setProcess(char processIdentifier) {this->processIdentifier = processIdentifier;}

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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::arc << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(center_x) << (int)(center_y) << (int)(radius) << \
             int(toDegrees(startAngle) * 16.f + 0.5f) << int(toDegrees(spanAngle) * 16.f + 0.5f) << \
             (char)(penWidth) << \
             (char)(penStyle) << ColorRGBA(penColor) << (char)(brushStyle) << ColorRGBA(brushColor)\
      ); \
    } \
  while(false)

/**
 * A macro that sends a ellipse
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::ellipse << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(center.x()) << (int)(center.y()) << (int)(radiusX) << (int)(radiusY) << (float)(rotation) << (char)(penWidth) << \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::rectangle << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(topLeft.x()) << (int)(topLeft.y()) << (int)(width) << (int)(height) << (float) rotation << (char)(penWidth) << \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OutTextSize _size; \
      for(int _i = 0; _i < numberOfPoints; ++_i) \
        _size << (int)(points[_i].x()) << (int)(points[_i].y()); \
      char* _buf = new char[_size.getSize() + 1]; \
      OutTextMemory _stream(_buf); \
      for(int _i = 0; _i < numberOfPoints; ++_i) \
        _stream << (int)(points[_i].x()) << (int)(points[_i].y()); \
      _buf[_size.getSize()] = 0; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::polygon << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)numberOfPoints << \
             _buf << \
             (char)(penWidth) << (char)(penStyle) << ColorRGBA(penColor) << \
             (char)(brushStyle) << ColorRGBA(brushColor) \
            );  \
      delete [] _buf; \
    } \
  while(false)

/**
 * A macro that sends a grid which has quadratic cells of different colors
 * @param id A drawing id (Drawings::FieldDrawing or Drawings::ImageDrawing)
 * @param x The x-coordinate of the grid center.
 * @param y The y-coordinate of the grid center.
 * @param cellSize The side length of a cell
 * @param cellsX The number of cells in x direction
 * @param cellsY The number of cells in y direction
 * @param cells An array containing the colors of the cells
 */
#define GRID_RGBA(id, x, y, cellSize, cellsX, cellsY, cells) \
  do \
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OutTextSize _size; \
      for(int _i = 0; _i < (cellsX*cellsY); ++_i) \
        _size << cells[_i]; \
      char* _buf = new char[_size.getSize() + 1]; \
      OutTextMemory _stream(_buf); \
      for(int _i = 0; _i < (cellsX*cellsY); ++_i) \
        _stream << cells[_i]; \
      _buf[_size.getSize()] = 0; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::gridRGBA << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             int(x) << \
             int(y) << \
             int(cellSize) << \
             int(cellsX) << \
             int(cellsY) << \
             _buf \
            ); \
      delete [] _buf; \
    } \
  while(false)

/**
 * A macro that sends a grid which has quadratic cells of different color intensity
 * @param id A drawing id (Drawings::FieldDrawing or Drawings::ImageDrawing)
 * @param x The x-coordinate of the grid center.
 * @param y The y-coordinate of the grid center.
 * @param cellSize The side length of a cell
 * @param cellsX The number of cells in x direction
 * @param cellsY The number of cells in y direction
 * @param baseColor A color for drawing. The RGB (not A) values will become multiplied by cell[i]/256
 * @param cells An array (of unsigned char) containing the color intensities of the cells
 */
#define GRID_MONO(id, x, y, cellSize, cellsX, cellsY, baseColor, cells) \
  do \
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OutTextSize _size; \
      for(int _i = 0; _i < (cellsX*cellsY); ++_i) \
        _size << cells[_i]; \
      char* _buf = new char[_size.getSize() + 1]; \
      OutTextMemory _stream(_buf); \
      for(int _i = 0; _i < (cellsX*cellsY); ++_i) \
        _stream << cells[_i]; \
      _buf[_size.getSize()] = 0; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::gridMono << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             int(x) << \
             int(y) << \
             int(cellSize) << \
             int(cellsX) << \
             int(cellsY) << \
             ColorRGBA(baseColor) << \
             _buf \
            ); \
      delete [] _buf; \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::dot << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(xy.x()) << (int)(xy.y()) << ColorRGBA(penColor) << ColorRGBA(brushColor) \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::midDot << \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::largeDot << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << ColorRGBA(penColor) << ColorRGBA(brushColor) \
            ); \
    } \
  while(false)

/**
 * A macro that sends a line
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::line << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x1) << (int)(y1) << (int)(x2) << (int)(y2) << (char)(penWidth) << (char)(penStyle) << ColorRGBA(penColor) \
            ); \
    } \
  while(false)

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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::arrow << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x1) << (int)(y1) << (int)(x2) << (int)(y2) << (char)(penWidth) << (char)(penStyle) << ColorRGBA(penColor) \
            ); \
    } \
  while(false)

/**
 * A macro that sends an arrow for a pose
 * @param id A drawing id
 * @param p A Pose2f describing the arrow
 * @param color The color of the line arrow (Drawings::Color)
 */
#define POSE_2D_SAMPLE(id, p, color) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      Pose2f current = p; current += Pose2f(-100,0); \
      LINE(id, int(current.translation.x()),int(current.translation.y()), int(p.translation.x()), int(p.translation.y()), \
           1, Drawings::solidPen, color); \
      current = p; current += Pose2f(-40,-40); \
      LINE(id, int(current.translation.x()), int(current.translation.y()), int(p.translation.x()), int(p.translation.y()), \
           1, Drawings::solidPen, color); \
      current = p; current += Pose2f(-40,40); \
      LINE(id, int(current.translation.x()),int(current.translation.y()),int(p.translation.x()), int(p.translation.y()), \
           1, Drawings::solidPen, color); \
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
      LINE(id, x+size, y+size, x-size, y-size, penWidth, penStyle, penColor); \
      LINE(id, x+size, y-size, x-size, y+size, penWidth, penStyle, penColor); \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OutTextRawSize size; \
      size << txt; \
      char* _buf = new char[size.getSize() + 1]; \
      OutTextRawMemory stream(_buf); \
      stream << txt; \
      _buf[size.getSize()] = 0; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::text << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << (short)(fontSize) << (ColorRGBA)(color) << _buf \
            ); \
      delete [] _buf; \
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
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::origin << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(x) << (int)(y) << (float)(angle) \
            ); \
    } \
  while(false)

/**
 * A macro that sends a tip (popup text)
 * @param id A drawing id
 * @param center_x The x coordinate of the center of the anchor area
 * @param center_y The y coordinate of the center of the anchor area
 * @param radius The radius of the anchor area
 * @param text The text (streaming is possible)
 */
#define TIP(id, center_x, center_y, radius, text) \
  do \
    DECLARED_DEBUG_RESPONSE("debug drawing:" id) \
    { \
      OutTextRawSize size; \
      size << text; \
      char* _buf = new char[size.getSize() + 1]; \
      OutTextRawMemory stream(_buf); \
      stream << text; \
      _buf[size.getSize()] = 0; \
      OUTPUT(idDebugDrawing, bin, \
             (char)Drawings::tip << \
             (char)Global::getDrawingManager().getDrawingId(id) << \
             (int)(center_x) << (int)(center_y) << (int)(radius) << _buf \
            ); \
      delete [] _buf; \
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
      Vector2f bodyPoints[4] = {Vector2f(55, 90), Vector2f(-55, 90), \
                                 Vector2f(-55, -90), Vector2f(55, -90)}; \
      Vector2f translation = p.translation; \
      float rotation = p.rotation; \
      for(int i = 0; i < 4; i++) \
        bodyPoints[i] = p * bodyPoints[i]; \
      Vector2f dirVec(200.f, 0.f); \
      dirVec.rotate(rotation); \
      dirVec += translation; \
      LINE(id, translation.x(), translation.y(), dirVec.x(), dirVec.y(), \
           20, Drawings::solidPen, ColorRGBA::white); \
      POLYGON(id, 4, bodyPoints, 20, Drawings::solidPen, \
              ColorRGBA::black, Drawings::solidBrush, color); \
      CIRCLE(id, translation.x(), translation.y(), 42, 0, \
             Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color); \
    } \
  while(false)

/**
 * A macro that sends a covariance ellipse
 * @param id A drawing id
 * @param cov The covariance matrix as Matrix2f
 * @param mean The mean value
 */
#define COVARIANCE2D(id, cov, mean) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      const float factor = 1.f; \
      const float cov102 = cov(1, 0) * cov(1, 0); \
      const float varianceDiff = cov(0, 0) - cov(1, 1); \
      const float varianceDiff2 = varianceDiff * varianceDiff; \
      const float varianceSum = cov(0, 0) + cov(1, 1); \
      const float root = std::sqrt(varianceDiff2 + 4.0f * cov102); \
      const float eigenValue1 = 0.5f * (varianceSum + root); \
      const float eigenValue2 = 0.5f * (varianceSum - root); \
      \
      const float axis1 = 2.0f * std::sqrt(factor * eigenValue1); \
      const float axis2 = 2.0f * std::sqrt(factor * eigenValue2); \
      const float angle = 0.5f * std::atan2(2.0f * cov(1, 0), varianceDiff); \
      \
      ELLIPSE(id, mean, std::sqrt(3.0f) * axis1, std::sqrt(3.0f) * axis2, angle, \
              10, Drawings::solidPen, ColorRGBA(100,100,255,100), Drawings::solidBrush, ColorRGBA(100,100,255,100)); \
      ELLIPSE(id, mean, std::sqrt(2.0f) * axis1, std::sqrt(2.0f) * axis2, angle, \
              10, Drawings::solidPen, ColorRGBA(150,150,100,100), Drawings::solidBrush, ColorRGBA(150,150,100,100)); \
      ELLIPSE(id, mean, axis1, axis2, angle, \
              10, Drawings::solidPen, ColorRGBA(255,100,100,100), Drawings::solidBrush, ColorRGBA(255,100,100,100)); \
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
