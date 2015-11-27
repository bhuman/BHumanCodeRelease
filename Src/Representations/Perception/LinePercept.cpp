/**
 * @file LinePercept.h
 * Implementation of a struct that represents the fieldline percepts
 * @author jeff
 */

#include "LinePercept.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Blackboard.h"

using namespace std;

Vector2f LinePercept::Line::calculateClosestPointOnLine(const Vector2f& p) const
{
  const Vector2f normal = Vector2f(cos(alpha + pi), sin(alpha + pi));
  return p + normal * calculateDistToLine(p);
}

float LinePercept::getClosestLine(Vector2f point, Line& retLine) const
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

void LinePercept::draw() const
{
  FieldDimensions* theFieldDimensions = nullptr;
  CameraInfo* theCameraInfo = nullptr;
  CameraMatrix* theCameraMatrix = nullptr;
  ImageCoordinateSystem* theImageCoordinateSystem = nullptr;

  if(Blackboard::getInstance().exists("FieldDimensions"))
    theFieldDimensions = static_cast<FieldDimensions*>(&(Blackboard::getInstance()["FieldDimensions"]));
  if(Blackboard::getInstance().exists("CameraInfo"))
    theCameraInfo = static_cast<CameraInfo*>(&(Blackboard::getInstance()["CameraInfo"]));
  if(Blackboard::getInstance().exists("CameraMatrix"))
    theCameraMatrix = static_cast<CameraMatrix*>(&(Blackboard::getInstance()["CameraMatrix"]));
  if(Blackboard::getInstance().exists("ImageCoordinateSystem"))
    theImageCoordinateSystem =  static_cast<ImageCoordinateSystem*>(&(Blackboard::getInstance()["ImageCoordinateSystem"]));

  if(theFieldDimensions == nullptr || theCameraInfo == nullptr || theCameraMatrix == nullptr || theImageCoordinateSystem == nullptr)
    return;

  DECLARE_DEBUG_DRAWING("representation:LinePercept:Field", "drawingOnField");
  COMPLEX_DRAWING("representation:LinePercept:Field")
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      const Drawings::PenStyle pen = line->midLine ? Drawings::dashedPen : Drawings::solidPen;
      LINE("representation:LinePercept:Field", line->first.x(), line->first.y(), line->last.x(), line->last.y(), 15, pen, ColorRGBA::red);
      ARROW("representation:LinePercept:Field", line->first.x(), line->first.y(), line->first.x() + cos(line->alpha - pi_2) * 100, line->first.y() + sin(line->alpha - pi_2) * 100, 15, pen, ColorRGBA::blue);
      CROSS("representation:LinePercept:Field", line->first.x(), line->first.y(), 10, 5, pen, ColorRGBA::red);
    }
    for(vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      switch(inter->type)
      {
        case Intersection::X:
          LINE("representation:LinePercept:Field", inter->pos.x() - inter->dir1.x() * 100, inter->pos.y() - inter->dir1.y() * 100,
               inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100,
               15, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:LinePercept:Field", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100,
               inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100,
               15, Drawings::solidPen, ColorRGBA::blue);
          break;
        case Intersection::T:
          LINE("representation:LinePercept:Field", inter->pos.x(), inter->pos.y(),
               inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100,
               15, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:LinePercept:Field", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100,
               inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100,
               15, Drawings::solidPen, ColorRGBA::blue);
          break;
        case Intersection::L:
          LINE("representation:LinePercept:Field", inter->pos.x(), inter->pos.y(),
               inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100,
               15, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:LinePercept:Field", inter->pos.x(), inter->pos.y(),
               inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100,
               15, Drawings::solidPen, ColorRGBA::blue);
          break;
      }
    }
    if(circle.found)
    {
      CROSS("representation:LinePercept:Field", circle.pos.x(), circle.pos.y(), 40, 40, Drawings::solidPen, ColorRGBA::blue);
      CIRCLE("representation:LinePercept:Field", circle.pos.x(), circle.pos.y(), theFieldDimensions->centerCircleRadius, 30, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
    }
  }

  DECLARE_DEBUG_DRAWING("representation:LinePercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:LinePercept:ImageText", "drawingOnImage");
  COMPLEX_DRAWING("representation:LinePercept:Image")
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      const Drawings::PenStyle pen = line->midLine ? Drawings::dashedPen : Drawings::solidPen;

      const Vector2f lineInImageDirection = line->endInImage - line->startInImage;
      const Vector2f offSet = Vector2f(5.f, -10.f);
      const Vector2f textPosition = line->startInImage + 0.5f * lineInImageDirection + offSet;
      DRAWTEXT("representation:LinePercept:ImageText", textPosition.x(), textPosition.y(), 8, ColorRGBA::red, "" << (line->first - line->last).norm() / 1000.f << "m");
      LINE("representation:LinePercept:Image", line->startInImage.x(), line->startInImage.y(), line->endInImage.x(), line->endInImage.y(), 3, pen, ColorRGBA::red);
    }

    if(circle.found)
    {
      Vector2f p1;
      if(Transformation::robotToImage(circle.pos, *theCameraMatrix, *theCameraInfo, p1))
      {
        Vector2f uncor = theImageCoordinateSystem->fromCorrectedLinearized(p1);
        CROSS("representation:LinePercept:Image", uncor.x(), uncor.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
      }
      const float stepSize = 0.2f;
      for(float i = 0; i < pi2; i += stepSize)
      {
        Vector2f p1 = Vector2f::Zero();
        Vector2f p2 = Vector2f::Zero();
        if(Transformation::robotToImage(Vector2f(circle.pos + Vector2f(theFieldDimensions->centerCircleRadius, 0).rotate(i)), *theCameraMatrix, *theCameraInfo, p1) &&
           Transformation::robotToImage(Vector2f(circle.pos + Vector2f(theFieldDimensions->centerCircleRadius, 0).rotate(i + stepSize)), *theCameraMatrix, *theCameraInfo, p2))
        {
          Vector2f uncor1 = theImageCoordinateSystem->fromCorrectedLinearized(p1);
          Vector2f uncor2 = theImageCoordinateSystem->fromCorrectedLinearized(p2);
          LINE("representation:LinePercept:Image", uncor1.x(), uncor1.y(), uncor2.x(), uncor2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
        }
      }
    }
    for(vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      Vector2f p1 = Vector2f::Zero();
      Vector2f p2 = Vector2f::Zero();
      Vector2f p3 = Vector2f::Zero();
      Vector2f p4 = Vector2f::Zero();
      const Vector2f dir1 = inter->dir1 * 100.f;
      const Vector2f dir2 = inter->dir2 * 100.f;
      bool transformationSuccessful = true;
      switch(inter->type)
      {
        case Intersection::X:
          transformationSuccessful = Transformation::robotToImage(Vector2f(inter->pos - dir1), *theCameraMatrix, *theCameraInfo, p1) &&
            Transformation::robotToImage(Vector2f(inter->pos + dir1), *theCameraMatrix, *theCameraInfo, p2) &&
            Transformation::robotToImage(Vector2f(inter->pos - dir2), *theCameraMatrix, *theCameraInfo, p3) &&
            Transformation::robotToImage(Vector2f(inter->pos + dir2), *theCameraMatrix, *theCameraInfo, p4);
          break;
        case Intersection::T:
          transformationSuccessful = Transformation::robotToImage(Vector2f(inter->pos), *theCameraMatrix, *theCameraInfo, p1) &&
            Transformation::robotToImage(Vector2f(inter->pos + dir1), *theCameraMatrix, *theCameraInfo, p2) &&
            Transformation::robotToImage(Vector2f(inter->pos - dir2), *theCameraMatrix, *theCameraInfo, p3) &&
            Transformation::robotToImage(Vector2f(inter->pos + dir2), *theCameraMatrix, *theCameraInfo, p4);
          break;
        case Intersection::L:
          transformationSuccessful = Transformation::robotToImage(Vector2f(inter->pos), *theCameraMatrix, *theCameraInfo, p1) &&
            Transformation::robotToImage(Vector2f(inter->pos + dir1), *theCameraMatrix, *theCameraInfo, p2) &&
            Transformation::robotToImage(Vector2f(inter->pos), *theCameraMatrix, *theCameraInfo, p3) &&
            Transformation::robotToImage(Vector2f(inter->pos + dir2), *theCameraMatrix, *theCameraInfo, p4);
          break;
      }
      if(transformationSuccessful)
      {
        const Vector2f uncor1 = theImageCoordinateSystem->fromCorrectedLinearized(p1);
        const Vector2f uncor2 = theImageCoordinateSystem->fromCorrectedLinearized(p2);
        const Vector2f uncor3 = theImageCoordinateSystem->fromCorrectedLinearized(p3);
        const Vector2f uncor4 = theImageCoordinateSystem->fromCorrectedLinearized(p4);
        ARROW("representation:LinePercept:Image", uncor1.x(), uncor1.y(), uncor2.x(), uncor2.y(),
              3, Drawings::solidPen, ColorRGBA::blue);
        ARROW("representation:LinePercept:Image", uncor3.x(), uncor3.y(), uncor4.x(), uncor4.y(),
              3, Drawings::solidPen, ColorRGBA::green);
        Vector2f intersectionInImage;
        if(Transformation::robotToImage(inter->pos, *theCameraMatrix, *theCameraInfo, intersectionInImage))
        {
          const Vector2f uncorIntersection = theImageCoordinateSystem->fromCorrectedLinearized(intersectionInImage);
          DRAWTEXT("representation:LinePercept:Image", uncorIntersection.x(), uncorIntersection.y(), 25, ColorRGBA(255, 180, 180),
                   (inter->type == Intersection::L ? "L" : inter->type == Intersection::T ? "T" : "X"));
        }
      }
    }
  }

  DECLARE_DEBUG_DRAWING3D("representation:LinePercept", "robot");
  TRANSLATE3D("representation:LinePercept", 0, 0, -210);
  COMPLEX_DRAWING3D("representation:LinePercept")
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
      LINE3D("representation:LinePercept", line->first.x(), line->first.y(), 0, line->last.x(), line->last.y(), 0, 2, ColorRGBA::red);
    for(vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      switch(inter->type)
      {
        case Intersection::X:
          LINE3D("representation:LinePercept", inter->pos.x() - inter->dir1.x() * 100, inter->pos.y() - inter->dir1.y() * 100, 0,
                 inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100, 0,
                 2, ColorRGBA::blue);
          LINE3D("representation:LinePercept", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100, 0,
                 inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100, 0,
                 2, ColorRGBA::blue);
          break;
        case Intersection::T:
          LINE3D("representation:LinePercept", inter->pos.x(), inter->pos.y(), 0,
                 inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100, 0,
                 2, ColorRGBA::blue);
          LINE3D("representation:LinePercept", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100, 0,
                 inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100, 0,
                 2, ColorRGBA::blue);
          break;
        case Intersection::L:
          LINE3D("representation:LinePercept", inter->pos.x(), inter->pos.y(), 0,
                 inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100, 0,
                 2, ColorRGBA::blue);
          LINE3D("representation:LinePercept", inter->pos.x(), inter->pos.y(), 0,
                 inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100, 0,
                 2, ColorRGBA::blue);
          break;
      }
    }
    if(circle.found)
    {
      Vector2f v1(circle.pos.x() + theFieldDimensions->centerCircleRadius, circle.pos.y());
      for(int i = 1; i < 33; ++i)
      {
        const float angle = i * pi2 / 32;
        const Vector2f v2(circle.pos.x() + cos(angle) * (theFieldDimensions->centerCircleRadius),
                          circle.pos.y() + sin(angle) * (theFieldDimensions->centerCircleRadius));
        LINE3D("representation:LinePercept", v1.x(), v1.y(), 0, v2.x(), v2.y(), 0, 2, ColorRGBA::blue);
        v1 = v2;
      }
    }
  }
}
