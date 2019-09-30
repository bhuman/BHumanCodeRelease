/**
 * @file Controller/Visualization/DebugDrawing.h
 * Declaration of class DebugDrawing.
 *
 * @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"

#include "Tools/MessageQueue/InMessage.h"

class DebugDrawing;

/**
 * The DebugDrawing class defines a class of drawing objects for debug purposes.
 *
 * The DebugDrawing object provides member functions for painting simple graphic structures like lines, ellipses, polygons and text.
 * To use a DebugDrawing object, construct it, and then call its member functions.
 *
 * @author Matthias Jüngel
 */
class DebugDrawing
{
public:
  enum class ElementType
  {
    arc, ellipse, line, origin, polygon, rectangle, text, tip, robot, spot
  };

  /** base class for all drawing elements */
  struct Element
  {
    const ElementType type;
    Drawings::PenStyle penStyle;
    ColorRGBA penColor;
    float width;

    Element(ElementType type) : type(type) {};
  };

  /** Stores an arc */
  struct Arc : public Element
  {
    Vector2i center;
    int radius;
    Angle startAngle, spanAngle;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;

    Arc() : Element(ElementType::arc) {}
  };

  /** Stores an ellipse */
  struct Ellipse : public Element
  {
    Vector2i center, radii;
    Angle rotation;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;

    Ellipse() : Element(ElementType::ellipse) {}
  };

  /** Stores a line */
  struct Line : public Element
  {
    Vector2f start, end;

    Line() : Element(ElementType::line) {}
  };

  /** Stores a new origin */
  struct Origin : public Element
  {
    Vector2i translation;
    Angle angle;

    Origin() : Element(ElementType::origin) {}
  };

  /** Stores a polygon */
  struct Polygon : public Element
  {
    int nCount;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;

    Polygon() : Element(ElementType::polygon) {}
  };

  /** Stores a Rectangle */
  struct Rectangle : public Element
  {
    int topLX, topLY, w, h;
    float rotation;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;

    Rectangle() : Element(ElementType::rectangle) {}
  };

  /** Stores a text */
  struct Text : public Element
  {
    int x, y, fontSize, size;

    Text() : Element(ElementType::text) {}
  };

  /** Stores a tip */
  class Tip : public Element
  {
  public:
    int x, y, radius, size, next;
    Tip() : Element(ElementType::tip) {}
  };

  /** Stores a robot */
  struct Robot : public Element
  {
    Pose2f p;
    Vector2f dirVec, dirHeadVec;
    float alphaRobot;
    ColorRGBA colorBody, colorDirVec, colorDirHeadVec;
    Robot() : Element(ElementType::robot) {}
  };

  /** Stores a spot */
  struct Spot : public Element
  {
    int x1, y1, x2, y2, size, next;
    Spot() : Element(ElementType::spot) {}
  };

  unsigned timestamp; /**< The time when this drawing was created. */
  std::string threadIdentifier; /**< The thread this drawing belongs to. */

  DebugDrawing();
  DebugDrawing(const DebugDrawing& other);
  DebugDrawing(const DebugDrawing* pDebugDrawing);
  ~DebugDrawing();

  /** The function empties the drawing. */
  void reset();

  /** Assignment operator.*/
  const DebugDrawing& operator=(const DebugDrawing& other);

  /** Adds the contents of another debug drawing to this one. */
  const DebugDrawing& operator+=(const DebugDrawing& other);

  /** Returns the tip text for a certain coordinate (or 0 if none exists). */
  const char* getTip(int& x, int& y, const Pose2f& origin) const;

  /** Returns the spot action for a certain coordinate (or 0 if none exists)*/
  const char* getSpot(int x, int y, const Pose2f& origin) const;

  /** Updates the origin if it was set in this drawing. */
  void updateOrigin(Pose2f& origin) const;

  /**
   * Adds an arrow to the debug drawing
   * @param start The starting point of the arrow
   * @param end The target of the arrow
   * @param penStyle Specifies the penStyle of the arrow.
   * @param width The width
   * @param color The color
   */
  void arrow(Vector2f start, Vector2f end, Drawings::PenStyle penStyle, float width, ColorRGBA color);

  /**
   * Adds a line to the debug drawing.
   * @param xStart Specifies the x-coordinate of the startpoint for the line.
   * @param yStart Specifies the y-coordinate of the startpoint for the line.
   * @param xEnd Specifies the x-coordinate of the endpoint for the line.
   * @param yEnd Specifies the y-coordinate of the endpoint for the line.
   * @param penStyle Specifies the penStyle of the Line.
   * @param width Specifies the width of the line.
   * @param color Specifies the color of the line.
   */
  void line(float xStart, float yStart, float xEnd, float yEnd, Drawings::PenStyle penStyle, float width, ColorRGBA color);

  /**
   * Adds a line to the debug drawing. The line is a solid black line with width 1.
   * @param xStart Specifies the x-coordinate of the startpoint for the line.
   * @param yStart Specifies the y-coordinate of the startpoint for the line.
   * @param xEnd Specifies the x-coordinate of the endpoint for the line.
   * @param yEnd Specifies the y-coordinate of the endpoint for the line.
   */
  void line(float xStart, float yStart, float xEnd, float yEnd);

  /**
   * Adds a polygon to the debug drawing.
   * @param points Points to an array of points that specifies the vertices of the polygon. Each point in the array is a Point.
   * @param nCount Specifies the number of vertices in the array.
   * @param width Specifies the width of the border.
   * @param penStyle Specifies the penStyle of the border.
   * @param penColor Specifies the color of the border.
   * @param brushStyle Specifies the brushStyle of the polygon.
   * @param brushColor Specifies the color of the polygon.
   */
  void polygon(const Vector2i* points, int nCount, int width, Drawings::PenStyle penStyle, ColorRGBA penColor, Drawings::BrushStyle brushStyle, ColorRGBA brushColor);

  /**
   * Adds a filled square to the debug drawing. The border of the square is a solid line with width 0.
   * The square is a 3x3 square.
   * @param x Specifies the center of the dot.
   * @param y Specifies the center of the dot.
   * @param penColor Specifies the penColor of the dot.
   * @param brushColor Specifies the brushColor of the dot.
   */
  void dot(int x, int y, ColorRGBA penColor, ColorRGBA brushColor);

  /**
   * Adds a filled square to the debug drawing. The border of the square is a solid line with width 5.
   * The square is a 10x10 square.
   * @param x Specifies the center of the dot.
   * @param y Specifies the center of the dot.
   * @param penColor Specifies the penColor of the dot.
   * @param brushColor Specifies the brushColor of the dot.
   */
  void largeDot(int x, int y, ColorRGBA penColor, ColorRGBA brushColor);

  /**
   * Adds a filled midsize dot to the debug drawing. The border of the square is a solid line with width 1.
   * @param x Specifies the center of the dot.
   * @param y Specifies the center of the dot.
   * @param penColor Specifies the penColor of the dot.
   * @param brushColor Specifies the brushColor of the dot.
   */
  void midDot(int x, int y, ColorRGBA penColor, ColorRGBA brushColor);

  /**
   * Adds a text to the debug drawing
   * @param text The text to be displayed.
   * @param x The x-coordinate of the upper left corner of the text.
   * @param y The y-coordinate of the upper left corner of the text.
   * @param fontSize The size of the font to render the text.
   * @param color The color of the text.
   */
  void text(const char* text, int x, int y, int fontSize, ColorRGBA color);

  /**
   * Adds a tip (popup region) to the debug drawing
   * @param text The text to be displayed.
   * @param x The x-coordinate of the center of the anchor region.
   * @param y The y-coordinate of the center of the anchor region.
   * @param radius The radius of the anchor region.
   */
  void tip(const char* text, int x, int y, int radius);

  /**
   * Adds a spot with action to the debug drawing
   *
   */
  void spot(const char* action, int x1, int y1, int x2, int y2);

  /**
   * Sets a new origin for further drawings
   * @param x Specifies the center of the new origin.
   * @param y Specifies the center of the new origin.
   * @param angle Specifies the orientation of the new origin.
   */
  void origin(int x, int y, float angle);

  /**
   * Adds a robot to the debug drawing
   * @param p The desired Pose2f
   * @param dirVec The direction vector of the body
   * @param dirHeadVec The direction vector of the head
   * @param alphaRobot The alpha of the robot to draw
   * @param colorBody The color of the robot
   * @param colorDirVec The color of the direction vector of the body (Set alpha channel to 0, to disable this drawing)
   * @param colorDirHeadVec The color of the direction vector of the head (Set alpha channel to 0, to disable this drawing)
   */
  void robot(Pose2f p, Vector2f dirVec, Vector2f dirHeadVec, float alphaRobot, ColorRGBA colorBody, ColorRGBA colorDirVec, ColorRGBA colorDirHeadVec);

  bool addShapeFromQueue(InMessage& message, Drawings::ShapeType shapeType);
  /**
   * The function returns a pointer to the first drawing element.
   * @return A pointer to the first drawing element or 0 if the drawing is empty.
   */
  const Element* getFirst() const
  {
    return usedSize > 0 ? reinterpret_cast<const Element*>(elements) : 0;
  }

  /**
   * The function returns a pointer to the next drawing element.
   * @param element A pointer to the drawing element the successor is requested of.
   * @return A pointer to the next drawing element or 0 if there is no further element.
   */
  const Element* getNext(const Element* element) const;

private:
  int usedSize; /**< The size of the element buffer actually used. */
  int reservedSize; /**< The reserved size of the element buffer. */
  char* elements; /**< Contains all elements of this debug drawing */
  int firstTip = -1; /**< The index of the first tip. */
  int lastOrigin = -1; /** The index of the last origin. */
  int firstSpot = -1; /**< The index of the last origin. */

  /**
   * The function reserves enough space in the element buffer to store a new element.
   * @param size The size of the new element in bytes.
   */
  void reserve(int size);

  /**
   * The function writes a number of bytes to the element buffer.
   * @param data The address of the bytes to be written.
   * @param size The number of bytes to be written.
   */
  void write(const void* data, int size);
};
