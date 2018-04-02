/**
 * @file RadarFieldLines.h
 *
 * @author apag
 */

#include "RadarView.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/Perception/FieldFeatures/MidCircle.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Blackboard.h"
#include <string>

void RadarView::draw() const
{
  CameraInfo* theCameraInfo = nullptr;
  CameraMatrix* theCameraMatrix = nullptr;
  MidCircle* theMidCircle = nullptr;

  if(Blackboard::getInstance().exists("CameraInfo"))
    theCameraInfo = static_cast<CameraInfo*>(&(Blackboard::getInstance()["CameraInfo"]));
  if(Blackboard::getInstance().exists("CameraMatrix"))
    theCameraMatrix = static_cast<CameraMatrix*>(&(Blackboard::getInstance()["CameraMatrix"]));
  if(Blackboard::getInstance().exists("MidCircle"))
    theMidCircle = static_cast<MidCircle*>(&(Blackboard::getInstance()["MidCircle"]));

  if(theCameraInfo == nullptr || theCameraMatrix == nullptr)
    return;

  DEBUG_DRAWING("representation:RadarView:field", "drawingOnField")
  {
    for(size_t i = 0; i < sortLines.size(); i++)
      for(std::vector<FieldLines::Line>::const_iterator line = sortLines.at(i).eqLines.begin(); line != sortLines.at(i).eqLines.end(); line++)
      {
        unsigned char num = (unsigned char) i;
        ColorRGBA color(255 / (unsigned char) sortLines.size() * num, 255 - (255 / (unsigned char) sortLines.size() * num), 255);
        const Drawings::PenStyle pen = line->midLine ? Drawings::dashedPen : Drawings::solidPen;
        LINE("representation:RadarView:field", line->first.x(), line->first.y(), line->last.x(), line->last.y(), 15, pen, color);
        ARROW("representation:RadarView:field", line->first.x(), line->first.y(), line->first.x() + cos(line->alpha - pi_2) * 100, line->first.y() + sin(line->alpha - pi_2) * 100, 15, pen, color);
        CROSS("representation:RadarView:field", line->first.x(), line->first.y(), 10, 5, pen, color);
      }

    for(std::vector<FieldLines::Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      const Drawings::PenStyle pen = line->midLine ? Drawings::dashedPen : Drawings::solidPen;
      LINE("representation:RadarView:field", line->first.x(), line->first.y(), line->last.x(), line->last.y(), 15, pen, (line->first - line->last).norm() <= 800 && theMidCircle->isValid ? ColorRGBA::magenta : ColorRGBA::yellow);
      ARROW("representation:RadarView:field", line->first.x(), line->first.y(), line->first.x() + cos(line->alpha - pi_2) * 100, line->first.y() + sin(line->alpha - pi_2) * 100, 15, pen, ColorRGBA::yellow);
      CROSS("representation:RadarView:field", line->first.x(), line->first.y(), 10, 5, pen, ColorRGBA::yellow);
      // DRAWTEXT("representation:RadarView:field", line->last.x(), line->last.y(), 70, ColorRGBA::yellow, std::to_string(line - lines.begin()));
    }
  }

  DECLARE_DEBUG_DRAWING("representation:RadarView:imageText", "drawingOnImage");
  DEBUG_DRAWING("representation:RadarView:image", "drawingOnImage")
  {
    for(std::vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
    {
      const Drawings::PenStyle pen = line->midLine ? Drawings::dashedPen : Drawings::solidPen;
      Vector2f pImg;
      if(Transformation::robotToImage(line->first, *theCameraMatrix, *theCameraInfo, pImg))
      {
        Vector2f startInImage = pImg;
        if(Transformation::robotToImage(line->last, *theCameraMatrix, *theCameraInfo, pImg))
        {
          Vector2f endInImage = pImg;
          const Vector2f lineInImageDirection = endInImage - startInImage;
          const Vector2f offSet = Vector2f(5.f, -10.f);
          const Vector2f textPosition = startInImage + 0.5f * lineInImageDirection + offSet;
          DRAWTEXT("representation:RadarView:imageText", textPosition.x(), textPosition.y(), 8, ColorRGBA::yellow, "" << (line->first - line->last).norm() / 1000.f << "m" << (line->last - line->first).angle() << "rad"); //TODO calcs rad in same direction
          LINE("representation:RadarView:image", startInImage.x(), startInImage.y(), endInImage.x(), endInImage.y(), 3, pen, ColorRGBA::yellow);
        }
      }
    }
  }

  DECLARE_DEBUG_DRAWING3D("representation:RadarView", "robot");
  TRANSLATE3D("representation:RadarView", 0, 0, -210);
  COMPLEX_DRAWING3D("representation:RadarView")
  {
    for(std::vector<Line>::const_iterator line = lines.begin(); line != lines.end(); line++)
      LINE3D("representation:RadarView", line->first.x(), line->first.y(), 0, line->last.x(), line->last.y(), 0, 2, ColorRGBA::red);
  }
}
