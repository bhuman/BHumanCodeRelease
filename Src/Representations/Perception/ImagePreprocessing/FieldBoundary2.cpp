/**
 * @author Alexis Tsogias
 */

#include "FieldBoundary2.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"

FieldBoundarySpots::Spot::Spot(Vector2i position, int rating) : position(position), rating(rating) {}

void FieldBoundarySpots::draw() const
{
  DEBUG_DRAWING("representation:FieldBoundarySpots:spot", "drawingOnImage")
  {
    for(const Spot& s : spots)
    {
      DRAWTEXT("representation:FieldBoundarySpots:spot", s.position.x() - 3, s.position.y() - 1, 3, ColorRGBA::cyan, s.rating);
      DOT("representation:FieldBoundarySpots:spot", s.position.x(), s.position.y(), ColorRGBA::cyan, ColorRGBA::cyan);
    }
  }
}

void FieldBoundary2::draw() const
{
  DEBUG_DRAWING("representation:FieldBoundary2:image", "drawingOnImage")
  {
    if(isValid)
    {
      const Vector2i* previ = nullptr;
      for(const Vector2i& p : boundaryInImage)
      {
        DOT("representation:FieldBoundary2:image", p.x(), p.y(), ColorRGBA::orange, ColorRGBA::orange);
        if(previ != nullptr)
        {
          LINE("representation:FieldBoundary2:image", p.x(), p.y(), previ->x(), previ->y(), 1, Drawings::solidPen, ColorRGBA::yellow);
        }
        previ = &p;
      }
    }
  }

  DEBUG_DRAWING("representation:FieldBoundary2:field", "drawingOnField")
  {
    if(isValid)
    {
      const Vector2f* prevf = nullptr;
      for(const Vector2f& p : boundaryOnField)
      {
        ASSERT(p.allFinite());
        DOT("representation:FieldBoundary2:field", p.x(), p.y(), ColorRGBA::black, ColorRGBA::black);
        if(prevf != nullptr)
        {
          LINE("representation:FieldBoundary2:field", p.x(), p.y(), prevf->x(), prevf->y(), 20, Drawings::solidPen, ColorRGBA::blue);
        }
        prevf = &p;
      }
    }
  }
}

int FieldBoundary2::getBoundaryY(int x) const
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

  double m = 1.0 * (right->y() - left->y()) / (right->x() - left->x());

  return static_cast<int>((x * m) + right->y() - (right->x() * m));
}
