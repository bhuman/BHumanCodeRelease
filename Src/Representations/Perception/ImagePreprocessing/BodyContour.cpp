/**
 * @file BodyContour.cpp
 * The file implements a struct that represents the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 * @author Thomas Röfer
 */

#include "BodyContour.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"

BodyContour::Line::Line(const Vector2i& p1, const Vector2i& p2) :
  p1(p1.x() < p2.x() ? p1 : p2), p2(p1.x() < p2.x() ? p2 : p1)
{}

void BodyContour::clipBottom(int x, int& y) const
{
  int yIntersection;
  for(auto i = lines.cbegin(); i != lines.cend(); ++i)
    if(i->yAt(x, yIntersection) && yIntersection < y)
      y = yIntersection;
}

void BodyContour::clipBottom(int x, int& y, int imageHeight) const
{
  clipBottom(x, y);

  //clippedY can be outside the image
  if(y < 0)
    y = 0;
  else if(y >= imageHeight)
    y = imageHeight - 1;
}

void BodyContour::clipLeft(int& x, int y) const
{
  int xIntersection;
  for(auto i = lines.cbegin(); i != lines.cend(); ++i)
    if(i->p1.y() > i->p2.y())
    {
      if(i->xAt(y, xIntersection) && xIntersection > x)
        x = xIntersection;
      else if(i->p2.y() <= y && i->p2.x() > x) // below a segment, clip anyway
        x = i->p2.x();
    }
}

void BodyContour::clipRight(int& x, int y) const
{
  int xIntersection;
  for(auto i = lines.cbegin(); i != lines.cend(); ++i)
    if(i->p1.y() < i->p2.y())
    {
      if(i->xAt(y, xIntersection) && xIntersection < x)
        x = xIntersection;
      else if(i->p1.y() <= y && i->p1.x() < x) // below a segment, clip anyway
        x = i->p1.x();
    }
}

int BodyContour::getLeftEdge(int y, int imageWidth) const
{
  int x = imageWidth;
  int xIntersection;
  for(const auto& line : lines)
  {
    if(line.p1.y() > line.p2.y())
    {
      if(line.xAt(y, xIntersection) && xIntersection < x && xIntersection > 0)
        x = xIntersection;
      else if(line.p1.y() <= y && line.p1.x() < x && line.p1.x() > 0) // below a segment, clip anyway
        x = line.p1.x();
    }
  }
  return x;
}

int BodyContour::getRightEdge(int y, int imageWidth) const
{
  int x = 0;
  int xIntersection;
  for(const auto& line : lines)
  {
    if(line.p1.y() < line.p2.y())
    {
      if(line.xAt(y, xIntersection) && xIntersection > x && xIntersection < imageWidth)
        x = xIntersection;
      else if(line.p2.y() <= y && line.p2.x() > x && line.p2.x() < imageWidth) // below a segment, clip anyway
        x = line.p2.x();
    }
  }
  return x;
}

void BodyContour::draw() const
{
  DEBUG_DRAWING("representation:BodyContour", "drawingOnImage")
  {
    for(const auto& line : lines)
    {
      if(line.p1.y() > line.p2.y())
        LINE("representation:BodyContour", line.p1.x(), line.p1.y(), line.p2.x(), line.p2.y(), 2,
             Drawings::solidPen, ColorRGBA(222, 12, 127));
      else
        LINE("representation:BodyContour", line.p1.x(), line.p1.y(), line.p2.x(), line.p2.y(), 2,
             Drawings::solidPen, ColorRGBA(127, 12, 222));
    }

    for(int x = 0; x < cameraResolution.x(); x += 10)
    {
      int y = cameraResolution.y();
      clipBottom(x, y);
      LINE("representation:BodyContour", x, y, x, cameraResolution.y(), 1,
           Drawings::solidPen, ColorRGBA(255, 0, 255));
    }
  }
  DEBUG_DRAWING("representation:BodyContour:maxY", "drawingOnImage")
  {
    int maxY = getMaxY();
    LINE("representation:BodyContour:maxY", 0, maxY, cameraResolution.x() - 1, maxY,
         1, Drawings::solidPen, ColorRGBA(255, 0, 255));
  }
}

int BodyContour::getMaxY() const
{
  int y = cameraResolution.y() - 1;
  clipBottom(0, y);
  clipBottom(cameraResolution.x() - 1, y);

  if(y < 0)
  {
    //no need to continue
    return 0;
  }

  for(const Line& line : lines)
  {
    if(line.p1.y() >= 0 && line.p1.x() >= 0 && line.p1.x() < cameraResolution.x() && line.p1.y() < y)
      y = line.p1.y();
    if(line.p2.y() >= 0 && line.p2.x() >= 0 && line.p2.x() < cameraResolution.x() && line.p2.y() < y)
      y = line.p2.y();
  }
  return y;
}

bool BodyContour::isValidPoint(const Vector2i& point) const
{
  Vector2i copy(point);
  clipBottom(copy.x(), copy.y());
  if(point.y() != copy.y())
    return false;

  clipLeft(copy.x(), copy.y());
  if(point.x() > copy.x())
    return false;
  if(point.x() < copy.x())
    return true;

  clipRight(copy.x(), copy.y());
  if(point.x() < copy.x())
    return false;

  return true;
}
