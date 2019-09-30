/**
 * @file FieldLines.cpp
 * Implementation of a struct that represents the fieldline percepts
 * @author jeff
 */

#include "FieldLines.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Blackboard.h"

using namespace std;

Vector2f FieldLines::Line::calculateClosestPointOnLine(const Vector2f& p) const
{
  const Vector2f normal = Vector2f(cos(alpha + pi), sin(alpha + pi));
  return p + normal * calculateDistToLine(p);
}

float FieldLines::getClosestLine(Vector2f point, Line& retLine) const
{
  vector<Line>::const_iterator closestLine = lines.end();
  float minDist = -1.f;
  for(vector<Line>::const_iterator l1 = lines.begin(); l1 != lines.end(); l1++)
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

void FieldLines::draw() const
{
  CameraInfo* theCameraInfo = nullptr;
  CameraMatrix* theCameraMatrix = nullptr;
  ImageCoordinateSystem* theImageCoordinateSystem = nullptr;

  if(Blackboard::getInstance().exists("CameraInfo"))
    theCameraInfo = static_cast<CameraInfo*>(&(Blackboard::getInstance()["CameraInfo"]));
  if(Blackboard::getInstance().exists("CameraMatrix"))
    theCameraMatrix = static_cast<CameraMatrix*>(&(Blackboard::getInstance()["CameraMatrix"]));
  if(Blackboard::getInstance().exists("ImageCoordinateSystem"))
    theImageCoordinateSystem = static_cast<ImageCoordinateSystem*>(&(Blackboard::getInstance()["ImageCoordinateSystem"]));

  if(theCameraInfo == nullptr || theCameraMatrix == nullptr || theImageCoordinateSystem == nullptr)
    return;

  DEBUG_DRAWING("representation:FieldLines:field", "drawingOnField")
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      const Drawings::PenStyle pen = line->midLine ? Drawings::dashedPen : Drawings::solidPen;
      LINE("representation:FieldLines:field", line->first.x(), line->first.y(), line->last.x(), line->last.y(), 15, pen, ColorRGBA::red);
      ARROW("representation:FieldLines:field", line->first.x(), line->first.y(), line->first.x() + cos(line->alpha - pi_2) * 100, line->first.y() + sin(line->alpha - pi_2) * 100, 15, pen, ColorRGBA::red);
      CROSS("representation:FieldLines:field", line->first.x(), line->first.y(), 10, 5, pen, ColorRGBA::red);
    }
  }

  DECLARE_DEBUG_DRAWING("representation:FieldLines:imageText", "drawingOnImage");
  DEBUG_DRAWING("representation:FieldLines:image", "drawingOnImage")
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      const Drawings::PenStyle pen = line->midLine ? Drawings::dashedPen : Drawings::solidPen;
      Vector2f pImg;
      if(Transformation::robotToImage(line->first, *theCameraMatrix, *theCameraInfo, pImg))
      {
        Vector2f startInImage = theImageCoordinateSystem->fromCorrected(pImg);
        if(Transformation::robotToImage(line->last, *theCameraMatrix, *theCameraInfo, pImg))
        {
          Vector2f endInImage = theImageCoordinateSystem->fromCorrected(pImg);
          const Vector2f lineInImageDirection = endInImage - startInImage;
          const Vector2f offSet = Vector2f(5.f, -10.f);
          const Vector2f textPosition = startInImage + 0.5f * lineInImageDirection + offSet;
          DRAWTEXT("representation:FieldLines:imageText", textPosition.x(), textPosition.y(), 8, ColorRGBA::red, "" << (line->first - line->last).norm() / 1000.f << "m" << (line->last - line->first).angle() << "rad"); //TODO calcs rad in same direction
          LINE("representation:FieldLines:image", startInImage.x(), startInImage.y(), endInImage.x(), endInImage.y(), 3, pen, ColorRGBA::red);
        }
      }
    }
  }

  DECLARE_DEBUG_DRAWING3D("representation:FieldLines", "robot");
  TRANSLATE3D("representation:FieldLines", 0, 0, -210);
  COMPLEX_DRAWING3D("representation:FieldLines")
  {
    for(vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
      LINE3D("representation:FieldLines", line->first.x(), line->first.y(), 0, line->last.x(), line->last.y(), 0, 2, ColorRGBA::red);
  }
}
