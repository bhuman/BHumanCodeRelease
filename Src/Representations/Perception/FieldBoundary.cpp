/**
* @author Alexis Tsogias
*/

#include "FieldBoundary.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Platform/BHAssert.h"

void FieldBoundary::draw() const
{
  if(width == 0)
    return;

  DECLARE_DEBUG_DRAWING("representation:FieldBoundary:BoundarySpots", "drawingOnImage");
  for(const Vector2<int>& p : boundarySpots)
  {
    DOT("representation:FieldBoundary:BoundarySpots", p.x, p.y, ColorClasses::blue, ColorClasses::blue);
  }

  DECLARE_DEBUG_DRAWING("representation:FieldBoundary:ConvexBoundary", "drawingOnImage");
  const Vector2<int>* previ = nullptr;
  for(const Vector2<int>& p : convexBoundary)
  {
    DOT_AS_VECTOR("representation:FieldBoundary:ConvexBoundary", p, ColorClasses::red, ColorClasses::red);
    if(previ != nullptr)
    {
      LINE("representation:FieldBoundary:ConvexBoundary", p.x, p.y, previ->x, previ->y, 1, Drawings::ps_solid, ColorClasses::red);
    }
    previ = &p;
  }

  int selected = -1;
  MODIFY("representation:FieldBoundary:SelectedCandidate", selected);

  DECLARE_DEBUG_DRAWING("representation:FieldBoundary:BoundaryCandidates", "drawingOnImage");
  int num = convexBoundaryCandidates.size();
  float step = 255.0f / (num - 1);
  int pos = 0;
  for(const InImage& tmpBoundary : convexBoundaryCandidates)
  {
    previ = nullptr;
    unsigned char colorMod = static_cast<unsigned char>(step * pos);
    ColorRGBA col = ColorRGBA(colorMod, colorMod, 255 - colorMod);
    if(pos == selected || selected < 0 || selected >= num)
    {
      for(const Vector2<int>& p : tmpBoundary)
      {
        DOT_AS_VECTOR("representation:FieldBoundary:BoundaryCandidates", p, col, col);
        if(previ != nullptr)
        {
          LINE("representation:FieldBoundary:BoundaryCandidates", p.x, p.y, previ->x, previ->y, 1, Drawings::ps_solid, col);
        }
        previ = &p;
      }
    }
    pos++;
  }

  DECLARE_DEBUG_DRAWING("representation:FieldBoundary:Image", "drawingOnImage");
  previ = nullptr;
  for(const Vector2<int>& p : boundaryInImage)
  {
    DOT_AS_VECTOR("representation:FieldBoundary:Image", p, ColorClasses::orange, ColorClasses::orange);
    if(previ != nullptr)
    {
      LINE("representation:FieldBoundary:Image", p.x, p.y, previ->x, previ->y, 1, Drawings::ps_solid, ColorClasses::orange);
    }
    previ = &p;
  }

  DECLARE_DEBUG_DRAWING("representation:FieldBoundary:Field", "drawingOnField");
  const Vector2<float>* prevf = nullptr;
  for(const Vector2<float>& p : boundaryOnField)
  {
    DOT("representation:FieldBoundary:Field", p.x, p.y, ColorClasses::black, ColorClasses::black);
    if(prevf != nullptr)
    {
      LINE("representation:FieldBoundary:Field", p.x, p.y, prevf->x, prevf->y, 20, Drawings::ps_solid, ColorClasses::black);
    }
    prevf = &p;
  }

  DECLARE_DEBUG_DRAWING("representation:FieldBoundary:HighestPoint", "drawingOnImage");
  LINE("representation:FieldBoundary:HighestPoint", highestPoint.x, highestPoint.y, highestPoint.x + 20, highestPoint.y, 2, Drawings::ps_solid, ColorClasses::black);
  LINE("representation:FieldBoundary:HighestPoint", highestPoint.x, highestPoint.y, highestPoint.x, highestPoint.y + 20, 2, Drawings::ps_solid, ColorClasses::black);
}

int FieldBoundary::getBoundaryY(int x) const
{
  ASSERT(boundaryInImage.size() >= 2);

  const Vector2<int>* left = &boundaryInImage.front();
  const Vector2<int>* right = &boundaryInImage.back();

  if(x < left->x)
    right = &(*(boundaryInImage.begin() + 1));
  else if(x > right->x)
    left = &(*(boundaryInImage.end() - 2));
  else
  {
    for(const Vector2<int>& point : boundaryInImage)
    {
      if(point.x == x)
        return point.y;
      else if(point.x < x && point.x > left->x)
        left = &point;
      else if(point.x > x && point.x < right->x)
        right = &point;
    }
  }

  double m = 1.0 * (right->y - left->y) / (right->x - left->x);

  return static_cast<int>((x * m) + right->y - (right->x * m));
}
