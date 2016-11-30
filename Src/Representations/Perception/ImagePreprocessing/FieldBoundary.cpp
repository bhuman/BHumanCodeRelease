/**
 * @author Alexis Tsogias
 */

#include "FieldBoundary.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"

void FieldBoundary::draw() const
{
  DEBUG_DRAWING("representation:FieldBoundary:boundarySpots", "drawingOnImage")
    for(const Vector2i& p : boundarySpots)
    {
      DOT("representation:FieldBoundary:boundarySpots", p.x(), p.y(), ColorRGBA::blue, ColorRGBA::blue);
    }

  DEBUG_DRAWING("representation:FieldBoundary:convexBoundary", "drawingOnImage")
  {
    const Vector2i* previ = nullptr;
    for(const Vector2i& p : convexBoundary)
    {
      DOT("representation:FieldBoundary:convexBoundary", p.x(), p.y(), ColorRGBA::red, ColorRGBA::red);
      if(previ != nullptr)
      {
        LINE("representation:FieldBoundary:convexBoundary", p.x(), p.y(), previ->x(), previ->y(), 1, Drawings::solidPen, ColorRGBA::red);
      }
      previ = &p;
    }
  }

  DEBUG_DRAWING("representation:FieldBoundary:image", "drawingOnImage")
    if(isValid && !boundaryInImage.empty() && !boundarySpots.empty())
    {
      const Vector2i* previ = nullptr;
      Vector2i point;
      if(boundaryInImage.front().x() > boundarySpots.front().x())
      {
        point = Vector2i(boundarySpots.front().x(), getBoundaryY(boundarySpots.front().x()));
        previ = &point;
        DOT("representation:FieldBoundary:image", point.x(), point.y(), ColorRGBA::orange, ColorRGBA::orange);
      }
      for(const Vector2i& p : boundaryInImage)
      {
        DOT("representation:FieldBoundary:image", p.x(), p.y(), ColorRGBA::orange, ColorRGBA::orange);
        if(previ != nullptr)
        {
          LINE("representation:FieldBoundary:image", p.x(), p.y(), previ->x(), previ->y(), 1, Drawings::solidPen, ColorRGBA::orange);
        }
        previ = &p;
      }
      if(boundaryInImage.back().x() < boundarySpots.back().x())
      {
        point = Vector2i(boundarySpots.back().x(), getBoundaryY(boundarySpots.back().x()));
        DOT("representation:FieldBoundary:image", point.x(), point.y(), ColorRGBA::orange, ColorRGBA::orange);
        if(previ != nullptr)
        {
          LINE("representation:FieldBoundary:image", point.x(), point.y(), previ->x(), previ->y(), 1, Drawings::solidPen, ColorRGBA::orange);
        }
      }
    }

  DEBUG_DRAWING("representation:FieldBoundary:field", "drawingOnField")
    if(isValid)
    {
      const Vector2f* prevf = nullptr;
      for(const Vector2f& p : boundaryOnField)
      {
        DOT("representation:FieldBoundary:field", p.x(), p.y(), ColorRGBA::black, ColorRGBA::black);
        if(prevf != nullptr)
        {
          LINE("representation:FieldBoundary:field", p.x(), p.y(), prevf->x(), prevf->y(), 20, Drawings::solidPen, ColorRGBA::black);
        }
        prevf = &p;
      }
    }
}

int FieldBoundary::getBoundaryY(int x) const
{
  ASSERT(boundaryInImage.size() >= 2);

  const Vector2i* left = &boundaryInImage.front();
  const Vector2i* right = &boundaryInImage.back();

  if(x < left->x())
    right = &(*(boundaryInImage.begin() + 1));
  else if(x > right->x())
    left = &(*(boundaryInImage.end() - 2));
  else
  {
    for(const Vector2i& point : boundaryInImage)
    {
      if(point.x() == x)
        return point.y();
      else if(point.x() < x && point.x() > left->x())
        left = &point;
      else if(point.x() > x && point.x() < right->x())
        right = &point;
    }
  }

  float m = static_cast<float>(right->y() - left->y()) / static_cast<float>(right->x() - left->x());

  return static_cast<int>(static_cast<float>(x - left->x()) * m) + left->y();
}
