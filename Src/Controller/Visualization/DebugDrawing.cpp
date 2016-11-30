/**
 * @file Controller/Visualization/DebugDrawing.cpp
 * Implementation of class DebugDrawing.
 *
 * @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 */

#include <cstring>
#include <limits>

#include "DebugDrawing.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

DebugDrawing::DebugDrawing()
{
  timeStamp = Time::getCurrentSystemTime();
  usedSize = reservedSize = 0;
  elements = 0;
}

const DebugDrawing& DebugDrawing::operator=(const DebugDrawing& other)
{
  reset();
  timeStamp = other.timeStamp;
  *this += other;
  return *this;
}

const DebugDrawing& DebugDrawing::operator+=(const DebugDrawing& other)
{
  int offset = usedSize;
  write(other.elements, other.usedSize);
  int i = other.firstTip;
  while(i != -1)
  {
    int address = offset + i;
    Tip& t = (Tip&) elements[address];
    i = t.next;
    t.next = firstTip;
    firstTip = address;
  }
  if(other.lastOrigin != -1)
    lastOrigin = offset + other.lastOrigin;
  return *this;
}

DebugDrawing::DebugDrawing(const DebugDrawing& other)
{
  usedSize = reservedSize = 0;
  elements = 0;
  *this = other;
}

DebugDrawing::DebugDrawing(const DebugDrawing* pDebugDrawing)
{
  usedSize = reservedSize = 0;
  elements = 0;
  *this = *pDebugDrawing;
}

DebugDrawing::~DebugDrawing()
{
  if(elements)
    free(elements);
}

void DebugDrawing::reset()
{
  timeStamp = Time::getCurrentSystemTime();
  usedSize = 0;
  firstTip = -1;
  lastOrigin = -1;
}

const char* DebugDrawing::getTip(int& x, int& y, const Pose2f& origin) const
{
  int i = firstTip;
  float minDiff2 = std::numeric_limits<float>::max();
  const char* text = nullptr;
  while(i != -1)
  {
    const Tip& t = (const Tip&) elements[i];
    Vector2f point = origin * Vector2f((float)t.x, (float)t.y);
    float diff2 = (Vector2f((float)x, (float) y) - point).squaredNorm();
    if(diff2 <= t.radius * t.radius && diff2 < minDiff2)
    {
      minDiff2 = diff2;
      x = t.x;
      y = t.y;
      text = (const char*) (&t + 1);
    }
    i = t.next;
  }
  return text;
}

void DebugDrawing::updateOrigin(Pose2f& origin) const
{
  if(lastOrigin != -1)
  {
    const Origin& o = (const Origin&) elements[lastOrigin];
    origin.translation.x() = (float)o.translation.x();
    origin.translation.y() = (float)o.translation.y();
    origin.rotation = o.angle;
  }
}

void DebugDrawing::arrow(Vector2f start, Vector2f end,
                         Drawings::PenStyle penStyle, int width, ColorRGBA color)
{
  Vector2f startToEnd((end.x() - start.x()) / 4, (end.y() - start.y()) / 4);
  Vector2f perpendicular(startToEnd.y(), -1 * startToEnd.x());
  // start to endpoint
  line((int)start.x(), (int)start.y(), (int)(end.x()), (int)(end.y()), penStyle, width, color);
  // endpoint to left and right
  line((int)(end.x()), (int)(end.y()), (int)(end.x() - startToEnd.x() + perpendicular.x()), (int)(end.y() - startToEnd.y() + perpendicular.y()), penStyle, width, color);
  line((int)(end.x()), (int)(end.y()), (int)(end.x() - startToEnd.x() - perpendicular.x()), (int)(end.y() - startToEnd.y() - perpendicular.y()), penStyle, width, color);
}

void DebugDrawing::text(const char* text, int x, int y, int fontSize, ColorRGBA color)
{
  Text element;
  element.x = x;
  element.y = y;
  element.fontSize = fontSize;
  element.penColor = color;
  element.size = (int)strlen(text) + 1;
  write(&element, sizeof(element));
  write(text, element.size);
}

void DebugDrawing::tip(const char* text, int x, int y, int radius)
{
  Tip element;
  element.x = x;
  element.y = y;
  element.radius = radius;
  element.size = (int)strlen(text) + 1;
  element.next = firstTip;
  firstTip = usedSize;
  write(&element, sizeof(element));
  write(text, element.size);
}

void DebugDrawing::line(int xStart, int yStart, int xEnd, int yEnd, Drawings::PenStyle penStyle, int width, ColorRGBA penColor)
{
  Line element;
  element.start = Vector2i(xStart, yStart);
  element.end = Vector2i(xEnd, yEnd);
  element.penStyle = penStyle;
  element.width = width;
  element.penColor = penColor;
  write(&element, sizeof(element));
}

void DebugDrawing::line(int xStart, int yStart, int xEnd, int yEnd)
{
  line(xStart, yStart, xEnd, yEnd, Drawings::solidPen, 1, ColorRGBA(0, 0, 0));
}

void DebugDrawing::polygon(const Vector2i* points, int nCount, int width, Drawings::PenStyle penStyle,
                           ColorRGBA penColor, Drawings::BrushStyle brushStyle, ColorRGBA brushColor)
{
  Polygon element;
  element.nCount = nCount;
  element.width = width;
  element.penStyle = penStyle;
  element.penColor = penColor;
  element.brushStyle = brushStyle;
  element.brushColor = brushColor;
  write(&element, sizeof(element));
  for(int i = 0; i < nCount; ++i) // avoid alignment problems
  {
    write(&points[i].x(), sizeof(int));
    write(&points[i].y(), sizeof(int));
  }
}

void DebugDrawing::dot(int x, int y, ColorRGBA penColor, ColorRGBA brushColor)
{
  Vector2i points[4];
  points[0].x() = x - 1;
  points[0].y() = y - 1;
  points[1].x() = x + 1;
  points[1].y() = y - 1;
  points[2].x() = x + 1;
  points[2].y() = y + 1;
  points[3].x() = x - 1;
  points[3].y() = y + 1;
  polygon(points, 4, 0, Drawings::solidPen, penColor, Drawings::solidBrush, brushColor);
}

void DebugDrawing::largeDot(int x, int y, ColorRGBA penColor, ColorRGBA brushColor)
{
  Vector2i points[4];
  points[0].x() = x - 25;
  points[0].y() = y - 25;
  points[1].x() = x + 25;
  points[1].y() = y - 25;
  points[2].x() = x + 25;
  points[2].y() = y + 25;
  points[3].x() = x - 25;
  points[3].y() = y + 25;
  polygon(points, 4, 3, Drawings::solidPen, penColor, Drawings::solidBrush, brushColor);
}

void DebugDrawing::midDot(int x, int y, ColorRGBA penColor, ColorRGBA brushColor)
{
  Vector2i points[4];
  points[0].x() = x - 2;
  points[0].y() = y - 2;
  points[1].x() = x + 2;
  points[1].y() = y - 2;
  points[2].x() = x + 2;
  points[2].y() = y + 2;
  points[3].x() = x - 2;
  points[3].y() = y + 2;
  polygon(points, 4, 1, Drawings::solidPen, penColor, Drawings::solidBrush, brushColor);
}

void DebugDrawing::origin(int x, int y, float angle)
{
  Origin element;
  element.translation = Vector2i(x, y);
  element.angle = angle;
  lastOrigin = usedSize;
  write(&element, sizeof(element));
}

bool DebugDrawing::addShapeFromQueue(InMessage& message, Drawings::ShapeType shapeType)
{
  switch((Drawings::ShapeType)shapeType)
  {
    case Drawings::circle:
    {
      Ellipse newCircle;
      char penWidth, penStyle, brushStyle;
      message.bin >> newCircle.center.x();
      message.bin >> newCircle.center.y();
      message.bin >> newCircle.radii.x();
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> newCircle.penColor;
      message.bin >> brushStyle;
      message.bin >> newCircle.brushColor;
      newCircle.width = penWidth;
      newCircle.penStyle = (Drawings::PenStyle)penStyle;
      newCircle.brushStyle = (Drawings::BrushStyle)brushStyle;
      newCircle.radii.y() = newCircle.radii.x();
      newCircle.rotation = 0.0f;
      write(&newCircle, sizeof(newCircle));
      break;
    }
    case Drawings::arc:
    {
      Arc newArc;
      char penWidth, penStyle, brushStyle;
      message.bin >> newArc.center.x();
      message.bin >> newArc.center.y();
      message.bin >> newArc.radius;
      message.bin >> newArc.startAngle;
      message.bin >> newArc.spanAngle;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> newArc.penColor;
      message.bin >> brushStyle;
      message.bin >> newArc.brushColor;
      newArc.width = penWidth;
      newArc.penStyle = (Drawings::PenStyle)penStyle;
      newArc.brushStyle = (Drawings::BrushStyle)brushStyle;
      write(&newArc, sizeof(newArc));
      break;
    }
    case Drawings::ellipse:
    {
      Ellipse newEllipse;
      char penWidth, penStyle, brushStyle;
      ColorRGBA penColor, brushColor;
      message.bin >> newEllipse.center.x();
      message.bin >> newEllipse.center.y();
      message.bin >> newEllipse.radii.x();
      message.bin >> newEllipse.radii.y();
      message.bin >> newEllipse.rotation;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> newEllipse.penColor;
      message.bin >> brushStyle;
      message.bin >> newEllipse.brushColor;
      newEllipse.width = penWidth;
      newEllipse.penStyle = (Drawings::PenStyle)penStyle;
      newEllipse.brushStyle = (Drawings::BrushStyle)brushStyle;
      write(&newEllipse, sizeof(newEllipse));
      break;
    }
    case Drawings::rectangle:
    {
      Rectangle newRect;
      char penWidth, penStyle, brushStyle;
      ColorRGBA penColor, brushColor;
      message.bin >> newRect.topLX;
      message.bin >> newRect.topLY;
      message.bin >> newRect.w;
      message.bin >> newRect.h;
      message.bin >> newRect.rotation;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> newRect.penColor;
      message.bin >> brushStyle;
      message.bin >> newRect.brushColor;
      newRect.width = penWidth;
      newRect.penStyle = (Drawings::PenStyle)penStyle;
      newRect.brushStyle = (Drawings::BrushStyle)brushStyle;
      write(&newRect, sizeof(newRect));
      break;
    }
    case Drawings::polygon:
    {
      int numberOfPoints;
      std::string buffer;
      message.bin >> numberOfPoints >> buffer;
      InTextMemory stream(buffer.c_str(), buffer.size());
      Vector2i* points = new Vector2i[numberOfPoints];
      for(int i = 0; i < numberOfPoints; ++i)
        stream >> points[i];
      char penWidth, penStyle, brushStyle;
      ColorRGBA brushColor, penColor;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> penColor;
      message.bin >> brushStyle;
      message.bin >> brushColor;
      this->polygon(points, numberOfPoints, penWidth,
                    (Drawings::PenStyle) penStyle, penColor,
                    (Drawings::BrushStyle) brushStyle, brushColor);
      delete[] points;
      break;
    }
    case Drawings::line:
    {
      int x1, y1, x2, y2;
      char penWidth, penStyle;
      ColorRGBA penColor;
      message.bin >> x1;
      message.bin >> y1;
      message.bin >> x2;
      message.bin >> y2;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> penColor;
      this->line(x1, y1, x2, y2, (Drawings::PenStyle)penStyle, penWidth, penColor);
      break;
    }
    case Drawings::origin:
    {
      int x, y;
      float angle;
      message.bin >> x;
      message.bin >> y;
      message.bin >> angle;
      this->origin(x, y, angle);
      break;
    }
    case Drawings::arrow:
    {
      int x1, y1, x2, y2;
      char penWidth, penStyle;
      ColorRGBA penColor;
      message.bin >> x1;
      message.bin >> y1;
      message.bin >> x2;
      message.bin >> y2;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> penColor;
      this->arrow(Vector2f((float)x1, (float)y1), Vector2f((float)x2, (float)y2), (Drawings::PenStyle)penStyle, penWidth, penColor);
      break;
    }
    case Drawings::dot:
    {
      int x, y;
      ColorRGBA penColor, brushColor;
      message.bin >> x;
      message.bin >> y;
      message.bin >> penColor;
      message.bin >> brushColor;
      this->dot(x, y, penColor, brushColor);
      break;
    }
    case Drawings::dotMedium:
    {
      int x, y;
      ColorRGBA penColor, brushColor;
      message.bin >> x;
      message.bin >> y;
      message.bin >> penColor;
      message.bin >> brushColor;
      this->midDot(x, y, penColor, brushColor);
      break;
    }
    case Drawings::dotLarge:
    {
      int x, y;
      ColorRGBA penColor, brushColor;
      message.bin >> x;
      message.bin >> y;
      message.bin >> penColor;
      message.bin >> brushColor;
      this->largeDot(x, y, penColor, brushColor);
      break;
    }
    case Drawings::text:
    {
      int x, y;
      short fontSize;
      ColorRGBA color;
      message.bin >> x;
      message.bin >> y;
      message.bin >> fontSize;
      message.bin >> color;
      std::string text;
      message.bin >> text;
      this->text(text.c_str(), x, y, fontSize, color);
      break;
    }
    case Drawings::tip:
    {
      int x, y, radius;
      message.bin >> x;
      message.bin >> y;
      message.bin >> radius;
      std::string text;
      message.bin >> text;
      this->tip(text.c_str(), x, y, radius);
      break;
    }
  }
  return true;
}

void DebugDrawing::reserve(int size)
{
  if(usedSize + size > reservedSize)
  {
    if(!reservedSize)
      reservedSize = 1024;
    while(usedSize + size > reservedSize)
      reservedSize *= 2;
    elements = (char*)realloc(elements, reservedSize);
  }
}

void DebugDrawing::write(const void* data, int size)
{
  reserve(size);
  memcpy(elements + usedSize, data, size);
  usedSize += size;
}

const DebugDrawing::Element* DebugDrawing::getNext(const Element* element) const
{
  switch(element->type)
  {
    case ElementType::arc:
      element = (const Element*)((const Arc*)element + 1);
      break;
    case ElementType::ellipse:
      element = (const Element*)((const Ellipse*)element + 1);
      break;
    case ElementType::line:
      element = (const Element*)((const Line*)element + 1);
      break;
    case ElementType::origin:
      element = (const Element*)((const Origin*)element + 1);
      break;
    case ElementType::polygon:
      element = (const Element*)((const int*)((const Polygon*)element + 1) + ((const Polygon*)element)->nCount * 2);
      break;
    case ElementType::rectangle:
      element = (const Element*)((const Rectangle*)element + 1);
      break;
    case ElementType::text:
      element = (const Element*)((const char*)element + sizeof(Text) + ((const Text*)element)->size);
      break;
    case ElementType::tip:
      element = (const Element*)((const char*) element + sizeof(Tip) + ((const Tip*)element)->size);
      break;
    default:
      ASSERT(false);
  }
  return (const char*)element - elements < usedSize ? element : 0;
}
