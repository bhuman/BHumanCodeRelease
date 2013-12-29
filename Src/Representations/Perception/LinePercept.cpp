/**
* @file LinePercept.h
* Implementation of a class that represents the fieldline percepts
* @author jeff
*/

#include "LinePercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Geometry.h"

using namespace std;

Vector2<> LinePercept::Line::calculateClosestPointOnLine(const Vector2<>& p) const
{
  const Vector2<> normale = Vector2<>(cos(alpha + pi), sin(alpha + pi));
  return p + normale * calculateDistToLine(p);
}

float LinePercept::getClosestLine(Vector2<> point, Line& retLine) const
{
  vector<LinePercept::Line>::const_iterator closestLine = lines.end();
  float minDist = -1.f;
  for(vector<LinePercept::Line>::const_iterator l1 = lines.begin(); l1 != lines.end(); l1++)
  {
    const float dist = abs(l1->calculateDistToLine(point));
    if(dist < minDist || minDist == -1)
    {
      closestLine = l1;
      minDist = dist;
    }
  }

  if(minDist != -1.f)
    retLine = *closestLine;
  return minDist;
}

void LinePercept::drawOnField(const FieldDimensions& theFieldDimensions, float circleBiggerThanSpecified) const
{
  DECLARE_DEBUG_DRAWING("representation:LinePercept:Field", "drawingOnField");
  COMPLEX_DRAWING("representation:LinePercept:Field",
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      const Drawings::PenStyle pen = line->midLine ? Drawings::ps_dash : Drawings::ps_solid;
      LINE("representation:LinePercept:Field", line->first.x, line->first.y, line->last.x, line->last.y, 15, pen, ColorClasses::red);
      ARROW("representation:LinePercept:Field", line->first.x, line->first.y, line->first.x + cos(line->alpha - pi_2) * 100, line->first.y + sin(line->alpha - pi_2) * 100, 15, pen, ColorClasses::blue);
      CROSS("representation:LinePercept:Field", line->first.x, line->first.y, 10, 5, pen, ColorClasses::red);
    }
    for(vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      switch(inter->type)
      {
      case Intersection::X:
        LINE("representation:LinePercept:Field", inter->pos.x - inter->dir1.x * 100, inter->pos.y - inter->dir1.y * 100,
             inter->pos.x + inter->dir1.x * 100, inter->pos.y + inter->dir1.y * 100,
             15, Drawings::ps_solid, ColorClasses::blue);
        LINE("representation:LinePercept:Field", inter->pos.x - inter->dir2.x * 100, inter->pos.y - inter->dir2.y * 100,
             inter->pos.x + inter->dir2.x * 100, inter->pos.y + inter->dir2.y * 100,
             15, Drawings::ps_solid, ColorClasses::blue);
        break;
      case Intersection::T:
        LINE("representation:LinePercept:Field", inter->pos.x, inter->pos.y,
             inter->pos.x + inter->dir1.x * 100, inter->pos.y + inter->dir1.y * 100,
             15, Drawings::ps_solid, ColorClasses::blue);
        LINE("representation:LinePercept:Field", inter->pos.x - inter->dir2.x * 100, inter->pos.y - inter->dir2.y * 100,
             inter->pos.x + inter->dir2.x * 100, inter->pos.y + inter->dir2.y * 100,
             15, Drawings::ps_solid, ColorClasses::blue);
        break;
      case Intersection::L:
        LINE("representation:LinePercept:Field", inter->pos.x, inter->pos.y,
             inter->pos.x + inter->dir1.x * 100, inter->pos.y + inter->dir1.y * 100,
             15, Drawings::ps_solid, ColorClasses::blue);
        LINE("representation:LinePercept:Field", inter->pos.x , inter->pos.y,
             inter->pos.x + inter->dir2.x * 100, inter->pos.y + inter->dir2.y * 100,
             15, Drawings::ps_solid, ColorClasses::blue);
        break;
      }
    }
    if(circle.found)
    {
      CROSS("representation:LinePercept:Field", circle.pos.x, circle.pos.y, 40, 40, Drawings::ps_solid, ColorClasses::blue);
      CIRCLE("representation:LinePercept:Field", circle.pos.x, circle.pos.y, theFieldDimensions.centerCircleRadius + circleBiggerThanSpecified, 30, Drawings::ps_solid, ColorClasses::blue, Drawings::bs_null, ColorClasses::blue);
    }
    for(vector<LineSegment>::const_iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++)
      ARROW("representation:LinePercept:Field", seg->p1.x, seg->p1.y, seg->p2.x, seg->p2.y, 10, Drawings::ps_solid, ColorClasses::orange);
  });
}

void LinePercept::drawOnImage(const CameraMatrix& theCameraMatrix, const CameraInfo& theCameraInfo, const FieldDimensions& theFieldDimensions, float circleBiggerThanSpecified, const ImageCoordinateSystem& theImageCoordinateSystem) const
{
  DECLARE_DEBUG_DRAWING("representation:LinePercept:Image", "drawingOnImage");
  COMPLEX_DRAWING("representation:LinePercept:Image",
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      Vector2<int> p1;
      Vector2<int> p2;
      if(Geometry::calculatePointInImage(line->first, theCameraMatrix, theCameraInfo, p1) &&
      Geometry::calculatePointInImage(line->last, theCameraMatrix, theCameraInfo, p2))
      {
        const Drawings::PenStyle pen = line->midLine ? Drawings::ps_dash : Drawings::ps_solid;
        Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedLinearized(p1);
        Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedLinearized(p2);
        LINE("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y, 3, pen, ColorClasses::red);
      }
    }

    if(circle.found)
    {
      Vector2<int> p1;
      if(Geometry::calculatePointInImage(circle.pos, theCameraMatrix, theCameraInfo, p1))
      {
        Vector2<> uncor = theImageCoordinateSystem.fromCorrectedLinearized(p1);
        CROSS("representation:LinePercept:Image", uncor.x, uncor.y, 5, 3, Drawings::ps_solid, ColorClasses::blue);
      }
      const float stepSize = 0.2f;
      for(float i = 0; i < pi2; i += stepSize)
      {
        Vector2<int> p1;
        Vector2<int> p2;
        if(Geometry::calculatePointInImage(circle.pos + Vector2<>(theFieldDimensions.centerCircleRadius + circleBiggerThanSpecified, 0).rotate(i), theCameraMatrix, theCameraInfo, p1) &&
           Geometry::calculatePointInImage(circle.pos + Vector2<>(theFieldDimensions.centerCircleRadius + circleBiggerThanSpecified, 0).rotate(i + stepSize), theCameraMatrix, theCameraInfo, p2))
        {
          Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedLinearized(p1);
          Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedLinearized(p2);
          LINE("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y, 3, Drawings::ps_solid, ColorClasses::blue);
        }
      }

    }

    for(vector<LineSegment>::const_iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++)
    {
      Vector2<int> p1;
      Vector2<int> p2;
      if(Geometry::calculatePointInImage(seg->p1, theCameraMatrix, theCameraInfo, p1) &&
         Geometry::calculatePointInImage(seg->p2, theCameraMatrix, theCameraInfo, p2))
      {
        Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedApprox(p1);
        Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedApprox(p2);
        LINE("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y, 2, Drawings::ps_solid, ColorClasses::orange);
      }
    }

    for(vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      Vector2<int> p1;
      Vector2<int> p2;
      Vector2<int> p3;
      Vector2<int> p4;
      const Vector2<> dir1 = inter->dir1 * 100.f;
      const Vector2<> dir2 = inter->dir2 * 100.f;
      switch(inter->type)
      {
      case Intersection::X:
        Geometry::calculatePointInImage(inter->pos - dir1, theCameraMatrix, theCameraInfo, p1);
        Geometry::calculatePointInImage(inter->pos + dir1, theCameraMatrix, theCameraInfo, p2);
        Geometry::calculatePointInImage(inter->pos - dir2, theCameraMatrix, theCameraInfo, p3);
        Geometry::calculatePointInImage(inter->pos + dir2, theCameraMatrix, theCameraInfo, p4);
        break;
      case Intersection::T:
        Geometry::calculatePointInImage(inter->pos, theCameraMatrix, theCameraInfo, p1);
        Geometry::calculatePointInImage(inter->pos + dir1, theCameraMatrix, theCameraInfo, p2);
        Geometry::calculatePointInImage(inter->pos - dir2, theCameraMatrix, theCameraInfo, p3);
        Geometry::calculatePointInImage(inter->pos + dir2, theCameraMatrix, theCameraInfo, p4);
        break;
      case Intersection::L:
        Geometry::calculatePointInImage(inter->pos, theCameraMatrix, theCameraInfo, p1);
        Geometry::calculatePointInImage(inter->pos + dir1, theCameraMatrix, theCameraInfo, p2);
        Geometry::calculatePointInImage(inter->pos, theCameraMatrix, theCameraInfo, p3);
        Geometry::calculatePointInImage(inter->pos + dir2, theCameraMatrix, theCameraInfo, p4);
        break;
      }

      Vector2<> uncor1 = theImageCoordinateSystem.fromCorrectedLinearized(p1);
      Vector2<> uncor2 = theImageCoordinateSystem.fromCorrectedLinearized(p2);
      Vector2<> uncor3 = theImageCoordinateSystem.fromCorrectedLinearized(p3);
      Vector2<> uncor4 = theImageCoordinateSystem.fromCorrectedLinearized(p4);
      ARROW("representation:LinePercept:Image", uncor1.x, uncor1.y, uncor2.x, uncor2.y,
            3, Drawings::ps_solid, ColorClasses::blue);
      ARROW("representation:LinePercept:Image", uncor3.x, uncor3.y, uncor4.x, uncor4.y,
            3, Drawings::ps_solid, ColorClasses::blue);
    }
  });
}

void LinePercept::drawIn3D(const FieldDimensions& theFieldDimensions, float circleBiggerThanSpecified) const
{
  DECLARE_DEBUG_DRAWING3D("representation:LinePercept", "origin");
  TRANSLATE3D("representation:LinePercept", 0, 0, -210);
  COMPLEX_DRAWING3D("representation:LinePercept",
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
      LINE3D("representation:LinePercept", line->first.x, line->first.y, 0, line->last.x, line->last.y, 0, 2, ColorClasses::red);
    for(vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      switch(inter->type)
      {
      case Intersection::X:
        LINE3D("representation:LinePercept", inter->pos.x - inter->dir1.x * 100, inter->pos.y - inter->dir1.y * 100, 0,
               inter->pos.x + inter->dir1.x * 100, inter->pos.y + inter->dir1.y * 100, 0,
               2, ColorClasses::blue);
        LINE3D("representation:LinePercept", inter->pos.x - inter->dir2.x * 100, inter->pos.y - inter->dir2.y * 100, 0,
               inter->pos.x + inter->dir2.x * 100, inter->pos.y + inter->dir2.y * 100, 0,
               2, ColorClasses::blue);
        break;
      case Intersection::T:
        LINE3D("representation:LinePercept", inter->pos.x, inter->pos.y, 0,
               inter->pos.x + inter->dir1.x * 100, inter->pos.y + inter->dir1.y * 100, 0,
               2, ColorClasses::blue);
        LINE3D("representation:LinePercept", inter->pos.x - inter->dir2.x * 100, inter->pos.y - inter->dir2.y * 100, 0,
               inter->pos.x + inter->dir2.x * 100, inter->pos.y + inter->dir2.y * 100, 0,
               2, ColorClasses::blue);
        break;
      case Intersection::L:
        LINE3D("representation:LinePercept", inter->pos.x, inter->pos.y, 0,
               inter->pos.x + inter->dir1.x * 100, inter->pos.y + inter->dir1.y * 100, 0,
               2, ColorClasses::blue);
        LINE3D("representation:LinePercept", inter->pos.x , inter->pos.y, 0,
               inter->pos.x + inter->dir2.x * 100, inter->pos.y + inter->dir2.y * 100, 0,
               2, ColorClasses::blue);
        break;
      }
    }
    if(circle.found)
    {
      Vector2<> v1(circle.pos.x + theFieldDimensions.centerCircleRadius + circleBiggerThanSpecified, circle.pos.y);
      for(int i = 1; i < 33; ++i)
      {
        const float angle(i * pi2 / 32);
        Vector2<> v2(circle.pos.x + cos(angle) * (theFieldDimensions.centerCircleRadius + circleBiggerThanSpecified),
                     circle.pos.y + sin(angle) * (theFieldDimensions.centerCircleRadius + circleBiggerThanSpecified));
        LINE3D("representation:LinePercept", v1.x, v1.y, 0, v2.x, v2.y, 0, 2, ColorClasses::blue);
        v1 = v2;
      }
    }
    for(vector<LineSegment>::const_iterator seg = singleSegs.begin(); seg != singleSegs.end(); seg++)
      LINE3D("representation:LinePercept", seg->p1.x, seg->p1.y, 0, seg->p2.x, seg->p2.y, 0, 2, ColorClasses::orange);
  });
}
