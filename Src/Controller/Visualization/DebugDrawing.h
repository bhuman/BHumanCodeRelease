/**
* @file Controller/Visualization/DebugDrawing.h
* Declaration of class DebugDrawing.
*
* @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"

#include "Tools/MessageQueue/InMessage.h"

class DebugDrawing;

/**
* Streaming operator that reads a DebugDrawing from a stream.
* @param stream The stream from which is read.
* @param debugDrawing The DebugDrawing object.
* @return The stream.
*/
In& operator>>(In& stream, DebugDrawing& debugDrawing);

/**
* Streaming operator that writes a DebugDrawing to a stream.
* @param stream The stream to write on.
* @param debugDrawing The DebugDrawing object.
* @return The stream.
*/
Out& operator<<(Out& stream, const DebugDrawing& debugDrawing);

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
  /** Default constructor. */
  DebugDrawing();

  /** Copy constructor for a DebugDrawing object. */
  DebugDrawing(const DebugDrawing& other);

  /** Copy constructor for a DebugDrawing object. */
  DebugDrawing(const DebugDrawing* pDebugDrawing);

  /** Destructor. */
  ~DebugDrawing();

  /** The function empties the drawing. */
  void reset();

  /** Assignment operator.*/
  const DebugDrawing& operator=(const DebugDrawing& other);

  /** Adds the contents of another debug drawing to this one. */
  const DebugDrawing& operator+=(const DebugDrawing& other);

  /** Returns the tip text for a certain coordinate (or 0 if none exists). */
  const char* getTip(int& x, int& y) const;

  /** base class for all drawing elements */
  class Element
  {
  public:
    enum {LINE, POLYGON, ELLIPSE, ARC, TIP, ORIGIN, TEXT, GRID_RGBA, GRID_MONO, RECTANGLE} type;
    Drawings::PenStyle penStyle;
    ColorRGBA penColor;
    int width;
  };

  /** Stores a polygon */
  class Polygon : public Element
  {
  public:
    int nCount;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;
    Polygon() {type = POLYGON;}
  };

  /** Stores a grid */
  class GridRGBA : public Element
  {
  public:
    int x;
    int y;
    int cellSize;
    int cellsX;
    int cellsY;
    GridRGBA() {type = GRID_RGBA;}
  };

  /** Stores a grid */
  class GridMono : public Element
  {
  public:
    int x;
    int y;
    int cellSize;
    int cellsX;
    int cellsY;
    ColorRGBA baseColor;
    GridMono() {type = GRID_MONO;}
  };

  /** Stores a tip */
  class Tip : public Element
  {
  public:
    int x, y,
        radius,
        size,
        next;
    Tip() { type = TIP; }
  };

  /** Stores a line */
  class Line : public Element
  {
  public:
    int xStart, yStart, xEnd, yEnd;
    Line() {type = LINE;}
  };

  /** Stores an ellipse */
  class Ellipse : public Element
  {
  public:
    int x, y, radiusX, radiusY;
    float rotation;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;
    Ellipse() {type = ELLIPSE;}
  };

  /** Stores an ellipse */
  class Arc : public Element
  {
  public:
    int x, y, radius;
    int startAngle, spanAngle;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;
    Arc() {type = ARC;}
  };

  /** Stores a Rectangle */
  class Rectangle : public Element
  {
  public:
    int topLX, topLY, w, h;
    float rotation;
    Drawings::BrushStyle brushStyle;
    ColorRGBA brushColor;
    Rectangle() {type = RECTANGLE;}
  };

  /** Stores a text */
  class Text : public Element
  {
  public:
    int x, y,
        fontSize,
        size;
    Text() { type = TEXT; }
  };

  /** Stores a new origin */
  class Origin : public Element
  {
  public:
    int x, y;
    float angle;
    Origin() {type = ORIGIN;}
  };

  /**
  * Adds an arrow to the debug drawing
  * @param start The starting point of the arrow
  * @param end The target of the arrow
  * @param penStyle Specifies the penStyle of the arrow.
  * @param width The width
  * @param color The color
  */
  void arrow(
    Vector2f start,
    Vector2f end,
    Drawings::PenStyle penStyle,
    int width,
    ColorRGBA color
  );

  /**
  * Adds a tip (popup region) to the debug drawing
  * @param text The text to be displayed.
  * @param x The x-coordinate of the center of the anchor region.
  * @param y The y-coordinate of the center of the anchor region.
  * @param radius The radius of the anchor region.
  */
  void tip(const char* text, int x, int y, int radius);

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
  void line(
    int xStart,
    int yStart,
    int xEnd,
    int yEnd,
    Drawings::PenStyle penStyle,
    int width,
    ColorRGBA color
  );

  /**
  * Adds a line to the debug drawing. The line is a solid black line with width 1.
  * @param xStart Specifies the x-coordinate of the startpoint for the line.
  * @param yStart Specifies the y-coordinate of the startpoint for the line.
  * @param xEnd Specifies the x-coordinate of the endpoint for the line.
  * @param yEnd Specifies the y-coordinate of the endpoint for the line.
  */
  void line(int xStart, int yStart, int xEnd, int yEnd);

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
  void polygon(
    const Vector2i* points,
    int nCount,
    int width,
    Drawings::PenStyle penStyle,
    ColorRGBA penColor,
    Drawings::BrushStyle brushStyle,
    ColorRGBA brushColor
  );

  void gridRGBA(int x, int y, int cellSize, int cellsX, int cellsY, ColorRGBA* cells);

  void gridMono(int x, int y, int cellSize, int cellsX, int cellsY, const ColorRGBA& baseColor, unsigned char* cells);

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
  void text(
    const char* text,
    int x, int y,
    int fontSize,
    ColorRGBA color
  );

  /**
  * Sets a new origin for further drawings
  * @param x Specifies the center of the new origin.
  * @param y Specifies the center of the new origin.
  * @param angle Specifies the orientation of the new origin.
  */
  void origin(int x, int y, float angle);

  bool addShapeFromQueue(InMessage& message, Drawings::ShapeType shapeType);

  /** the kind of the drawing */
  int typeOfDrawing;

  unsigned timeStamp; /**< The time when this drawing was created. */
  char processIdentifier; /**< The process this drawing was received from. */

  /**
  * The function returns a pointer to the first drawing element.
  * @return A pointer to the first drawing element or 0 if the drawing is empty.
  */
  const Element* getFirst() const {return usedSize > 0 ? (const Element*) elements : 0;}

  /**
  * The function returns a pointer to the next drawing element.
  * @param element A pointer to the drawing element the successor is requested of.
  * @return A pointer to the next drawing element or 0 if there is no further element.
  */
  const Element* getNext(const Element* element) const;

private:
  int usedSize, /**< The size of the element buffer actually used. */
      reservedSize; /**< The reserved size of the element buffer. */
  char* elements; /**< Contains all elements of this debug drawing */
  int firstTip; /**< The index of the first tip. */

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

  friend In& operator>>(In& stream, DebugDrawing& debugDrawing);
  friend Out& operator<<(Out& stream, const DebugDrawing& debugDrawing);
};
