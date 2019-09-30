/**
 * @file Controller/Visualization/DebugDrawing.cpp
 * Implementation of class DebugDrawing.
 *
 * @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include <cstring>
#include <limits>

#include "DebugDrawing.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

DebugDrawing::DebugDrawing()
{
  timestamp = Time::getCurrentSystemTime();
  usedSize = reservedSize = 0;
  elements = 0;
}

const DebugDrawing& DebugDrawing::operator=(const DebugDrawing& other)
{
  reset();
  timestamp = other.timestamp;
  threadIdentifier = other.threadIdentifier;
  *this += other;
  return *this;
}

const DebugDrawing& DebugDrawing::operator+=(const DebugDrawing& other)
{
  int offset = usedSize;
  write(other.elements, other.usedSize);

  // logic for Spot
  int i = other.firstSpot;
  while(i != -1)
  {
    int address = offset + i;
    Spot& t = reinterpret_cast<Spot&>(elements[address]);
    i = t.next;
    t.next = firstSpot;
    firstSpot = address;
  }

  // logic for Tip
  i = other.firstTip;
  while(i != -1)
  {
    int address = offset + i;
    Tip& t = reinterpret_cast<Tip&>(elements[address]);
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
  timestamp = Time::getCurrentSystemTime();
  usedSize = 0;
  firstTip = -1;
  firstSpot = -1;
  lastOrigin = -1;
}

const char* DebugDrawing::getSpot(int x, int y, const Pose2f& origin) const
{
  int i = firstSpot;
  const char* action = nullptr;
  while(i != -1)
  {
    const Spot& s = reinterpret_cast<const Spot&>(elements[i]);
    Vector2f p1 = origin * Vector2f(static_cast<float>(s.x1), static_cast<float>(s.y1));
    Vector2f p2 = origin * Vector2f(static_cast<float>(s.x2), static_cast<float>(s.y2));
    if(p1.x() <= x && p1.y() <= y && x <= p2.x() && y <= p2.y())
    {
      action = reinterpret_cast<const char*>(&s + 1);
      return action;
    }

    i = s.next;
  }
  return action;
}

const char* DebugDrawing::getTip(int& x, int& y, const Pose2f& origin) const
{
  int i = firstTip;
  float minDiff2 = std::numeric_limits<float>::max();
  const char* text = nullptr;
  while(i != -1)
  {
    const Tip& t = reinterpret_cast<const Tip&>(elements[i]);
    Vector2f point = origin * Vector2f(static_cast<float>(t.x), static_cast<float>(t.y));
    float diff2 = (Vector2f(static_cast<float>(x), static_cast<float>(y)) - point).squaredNorm();
    if(diff2 <= t.radius * t.radius && diff2 < minDiff2)
    {
      minDiff2 = diff2;
      x = t.x;
      y = t.y;
      text = reinterpret_cast<const char*>(&t + 1);
    }
    i = t.next;
  }
  return text;
}

void DebugDrawing::updateOrigin(Pose2f& origin) const
{
  if(lastOrigin != -1)
  {
    const Origin& o = reinterpret_cast<const Origin&>(elements[lastOrigin]);
    origin.translation.x() = static_cast<float>(o.translation.x());
    origin.translation.y() = static_cast<float>(o.translation.y());
    origin.rotation = o.angle;
  }
}

void DebugDrawing::arrow(Vector2f start, Vector2f end,
                         Drawings::PenStyle penStyle, float width, ColorRGBA color)
{
  Vector2f startToEnd((end.x() - start.x()) / 4, (end.y() - start.y()) / 4);
  Vector2f perpendicular(startToEnd.y(), -1 * startToEnd.x());
  // start to endpoint
  line(start.x(), start.y(), end.x(), end.y(), penStyle, width, color);
  // endpoint to left and right
  line(end.x(), end.y(), end.x() - startToEnd.x() + perpendicular.x(), end.y() - startToEnd.y() + perpendicular.y(), penStyle, width, color);
  line(end.x(), end.y(), end.x() - startToEnd.x() - perpendicular.x(), end.y() - startToEnd.y() - perpendicular.y(), penStyle, width, color);
}

void DebugDrawing::text(const char* text, int x, int y, int fontSize, ColorRGBA color)
{
  Text element;
  element.x = x;
  element.y = y;
  element.fontSize = fontSize;
  element.penColor = color;
  element.size = static_cast<int>(strlen(text)) + 1;
  write(&element, sizeof(element));
  write(text, element.size);
}

void DebugDrawing::spot(const char* action, int x1, int y1, int x2, int y2)
{
  Spot element;
  element.x1 = x1;
  element.y1 = y1;
  element.x2 = x2;
  element.y2 = y2;
  element.size = static_cast<int>(strlen(action)) + 1;
  element.next = firstSpot;
  firstSpot = usedSize;
  write(&element, sizeof(element));
  write(action, element.size);
}

void DebugDrawing::tip(const char* text, int x, int y, int radius)
{
  Tip element;
  element.x = x;
  element.y = y;
  element.radius = radius;
  element.size = static_cast<int>(strlen(text)) + 1;
  element.next = firstTip;
  firstTip = usedSize;
  write(&element, sizeof(element));
  write(text, element.size);
}

void DebugDrawing::line(float xStart, float yStart, float xEnd, float yEnd, Drawings::PenStyle penStyle, float width, ColorRGBA penColor)
{
  Line element;
  element.start = Vector2f(xStart, yStart);
  element.end = Vector2f(xEnd, yEnd);
  element.penStyle = penStyle;
  element.width = width;
  element.penColor = penColor;
  write(&element, sizeof(element));
}

void DebugDrawing::line(float xStart, float yStart, float xEnd, float yEnd)
{
  line(xStart, yStart, xEnd, yEnd, Drawings::solidPen, 1, ColorRGBA(0, 0, 0));
}

void DebugDrawing::polygon(const Vector2i* points, int nCount, int width, Drawings::PenStyle penStyle,
                           ColorRGBA penColor, Drawings::BrushStyle brushStyle, ColorRGBA brushColor)
{
  Polygon element;
  element.nCount = nCount;
  element.width = static_cast<float>(width);
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

void DebugDrawing::robot(Pose2f p, Vector2f dirVec, Vector2f dirHeadVec, float alphaRobot, ColorRGBA colorBody, ColorRGBA colorDirVec, ColorRGBA colorDirHeadVec)
{
  Robot element;
  element.p = p;
  element.dirVec = dirVec;
  element.dirHeadVec = dirHeadVec;
  element.alphaRobot = alphaRobot;
  element.colorBody = colorBody;
  element.colorDirVec = colorDirVec;
  element.colorDirHeadVec = colorDirHeadVec;
  write(&element, sizeof(element));
}

bool DebugDrawing::addShapeFromQueue(InMessage& message, Drawings::ShapeType shapeType)
{
  switch(static_cast<Drawings::ShapeType>(shapeType))
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
      newCircle.penStyle = static_cast<Drawings::PenStyle>(penStyle);
      newCircle.brushStyle = static_cast<Drawings::BrushStyle>(brushStyle);
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
      newArc.penStyle = static_cast<Drawings::PenStyle>(penStyle);
      newArc.brushStyle = static_cast<Drawings::BrushStyle>(brushStyle);
      write(&newArc, sizeof(newArc));
      break;
    }
    case Drawings::ellipse:
    {
      Ellipse newEllipse;
      char penWidth, penStyle, brushStyle;
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
      newEllipse.penStyle = static_cast<Drawings::PenStyle>(penStyle);
      newEllipse.brushStyle = static_cast<Drawings::BrushStyle>(brushStyle);
      write(&newEllipse, sizeof(newEllipse));
      break;
    }
    case Drawings::rectangle:
    {
      Rectangle newRect;
      char penWidth, penStyle, brushStyle;
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
      newRect.penStyle = static_cast<Drawings::PenStyle>(penStyle);
      newRect.brushStyle = static_cast<Drawings::BrushStyle>(brushStyle);
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
                    static_cast<Drawings::PenStyle>(penStyle), penColor,
                    static_cast<Drawings::BrushStyle>(brushStyle), brushColor);
      delete[] points;
      break;
    }
    case Drawings::line:
    {
      float x1, y1, x2, y2, penWidth;
      char penStyle;
      ColorRGBA penColor;
      message.bin >> x1;
      message.bin >> y1;
      message.bin >> x2;
      message.bin >> y2;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> penColor;
      this->line(x1, y1, x2, y2, static_cast<Drawings::PenStyle>(penStyle), penWidth, penColor);
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
      float x1, y1, x2, y2, penWidth;
      char penStyle;
      ColorRGBA penColor;
      message.bin >> x1;
      message.bin >> y1;
      message.bin >> x2;
      message.bin >> y2;
      message.bin >> penWidth;
      message.bin >> penStyle;
      message.bin >> penColor;
      this->arrow(Vector2f(x1, y1), Vector2f(x2, y2),
                  static_cast<Drawings::PenStyle>(penStyle), penWidth, penColor);
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
    case Drawings::robot:
    {
      Pose2f p;
      Vector2f dirVec, dirHeadVec;
      float alphaRobot;
      ColorRGBA colorBody, colorDirVec, colorDirHeadVec;
      message.bin >> p;
      message.bin >> dirVec;
      message.bin >> dirHeadVec;
      message.bin >> alphaRobot;
      message.bin >> colorBody;
      message.bin >> colorDirVec;
      message.bin >> colorDirHeadVec;
      this->robot(p, dirVec, dirHeadVec, alphaRobot, colorBody, colorDirVec, colorDirHeadVec);
      break;
    }
    case Drawings::spot:
    {
      int x1, y1, x2, y2;
      std::string action;
      message.bin >> x1;
      message.bin >> y1;
      message.bin >> x2;
      message.bin >> y2;
      message.bin >> action;
      this->spot(action.c_str(), x1, y1, x2, y2);
      break;
    }
    case Drawings::thread:
    {
      message.bin >> threadIdentifier;
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
    elements = static_cast<char*>(realloc(elements, reservedSize));
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
      element = static_cast<const Element*>(static_cast<const Arc*>(element) + 1);
      break;
    case ElementType::ellipse:
      element = static_cast<const Element*>(static_cast<const Ellipse*>(element) + 1);
      break;
    case ElementType::line:
      element = static_cast<const Element*>(static_cast<const Line*>(element) + 1);
      break;
    case ElementType::origin:
      element = static_cast<const Element*>(static_cast<const Origin*>(element) + 1);
      break;
    case ElementType::polygon:
      element = reinterpret_cast<const Element*>(reinterpret_cast<const int*>(static_cast<const Polygon*>(element) + 1) + static_cast<const Polygon*>(element)->nCount * 2);
      break;
    case ElementType::rectangle:
      element = static_cast<const Element*>(static_cast<const Rectangle*>(element) + 1);
      break;
    case ElementType::text:
      element = reinterpret_cast<const Element*>(reinterpret_cast<const char*>(element) + sizeof(Text) + static_cast<const Text*>(element)->size);
      break;
    case ElementType::tip:
      element = reinterpret_cast<const Element*>(reinterpret_cast<const char*>(element) + sizeof(Tip) + static_cast<const Tip*>(element)->size);
      break;
    case ElementType::robot:
      element = static_cast<const Element*>(static_cast<const Robot*>(element) + 1);
      break;
    case ElementType::spot:
      element = reinterpret_cast<const Element*>(reinterpret_cast<const char*>(element) + sizeof(Spot) + static_cast<const Spot*>(element)->size);
      break;
    default:
      ASSERT(false);
  }
  return reinterpret_cast<const char*>(element) - elements < usedSize ? element : 0;
}
