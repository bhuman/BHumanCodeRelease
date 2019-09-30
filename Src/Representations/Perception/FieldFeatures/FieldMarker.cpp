/**
 * @file FieldMarker.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldMarker.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Module/Blackboard.h"

void MarkedLine::draw() const
{
  CameraInfo* theCameraInfo = nullptr;
  CameraMatrix* theCameraMatrix = nullptr;
  FieldLines* theFieldLines = nullptr;
  ImageCoordinateSystem* theImageCoordinateSystem = nullptr;

  if(Blackboard::getInstance().exists("CameraInfo"))
    theCameraInfo = static_cast<CameraInfo*>(&(Blackboard::getInstance()["CameraInfo"]));
  if(Blackboard::getInstance().exists("CameraMatrix"))
    theCameraMatrix = static_cast<CameraMatrix*>(&(Blackboard::getInstance()["CameraMatrix"]));
  if(Blackboard::getInstance().exists("ImageCoordinateSystem"))
    theImageCoordinateSystem = static_cast<ImageCoordinateSystem*>(&(Blackboard::getInstance()["ImageCoordinateSystem"]));
  if(Blackboard::getInstance().exists("FieldLines"))
    theFieldLines = static_cast<FieldLines*>(&(Blackboard::getInstance()["FieldLines"]));

  if(theCameraInfo == nullptr || theCameraMatrix == nullptr || theImageCoordinateSystem == nullptr || theFieldLines == nullptr)
    return;

  const FieldLines::Line& line = theFieldLines->lines[lineIndex];

  COMPLEX_DRAWING("representation:MarkedField:field")
  {
    const Drawings::PenStyle pen = Drawings::dottedPen;
    LINE("representation:MarkedField:field", line.first.x(), line.first.y(), line.last.x(), line.last.y(), 15, pen, ColorRGBA::blue);
    ARROW("representation:MarkedField:field", line.first.x(), line.first.y(), line.first.x() + cos(line.alpha - pi_2) * 100, line.first.y() + sin(line.alpha - pi_2) * 100, 15, pen, ColorRGBA::blue);
    CROSS("representation:MarkedField:field", line.first.x(), line.first.y(), 10, 5, pen, ColorRGBA::blue);
  }

  COMPLEX_DRAWING("representation:MarkedField:image")
  {
    Vector2f pImg;
    if(Transformation::robotToImage(line.first, *theCameraMatrix, *theCameraInfo, pImg))
    {
      Vector2f startInImage = theImageCoordinateSystem->fromCorrected(pImg);
      if(Transformation::robotToImage(line.last, *theCameraMatrix, *theCameraInfo, pImg))
      {
        Vector2f endInImage = theImageCoordinateSystem->fromCorrected(pImg);
        const Vector2f lineInImageDirection = endInImage - startInImage;
        const Vector2f offSet = Vector2f(5.f, -10.f);
        const Vector2f textPosition = startInImage + 0.5f * lineInImageDirection + offSet;
        DRAWTEXT("representation:MarkedField:imageText", textPosition.x(), textPosition.y(), 8, ColorRGBA::blue, "Marked with:" << marker);
        LINE("representation:MarkedField:image", startInImage.x(), startInImage.y(), endInImage.x(), endInImage.y(), 3, Drawings::dottedPen, ColorRGBA::blue);
      }
    }
  }

  TRANSLATE3D("representation:MarkedField", 0, 0, -210);
  LINE3D("representation:MarkedField", line.first.x(), line.first.y(), 0, line.last.x(), line.last.y(), 0, 2, ColorRGBA::blue);
}

MarkedLine::LineMarker MarkedLine::mirror(const MarkedLine::LineMarker marker)
{
  if(!marker)
    return marker;
  return LineMarker(marker + (marker < firstLineMarkerOther ? firstLineMarkerOther - 1 : -firstLineMarkerOther + 1));
};

MarkedLine::LineMarker MarkedLine::mirror() const
{
  return mirror(marker);
};

void MarkedIntersection::draw() const
{
  CameraInfo* theCameraInfo = nullptr;
  CameraMatrix* theCameraMatrix = nullptr;
  ImageCoordinateSystem* theImageCoordinateSystem = nullptr;
  FieldLineIntersections* theFieldLineIntersections = nullptr;

  if(Blackboard::getInstance().exists("CameraInfo"))
    theCameraInfo = static_cast<CameraInfo*>(&(Blackboard::getInstance()["CameraInfo"]));
  if(Blackboard::getInstance().exists("CameraMatrix"))
    theCameraMatrix = static_cast<CameraMatrix*>(&(Blackboard::getInstance()["CameraMatrix"]));
  if(Blackboard::getInstance().exists("ImageCoordinateSystem"))
    theImageCoordinateSystem = static_cast<ImageCoordinateSystem*>(&(Blackboard::getInstance()["ImageCoordinateSystem"]));
  if(Blackboard::getInstance().exists("FieldLineIntersections"))
    theFieldLineIntersections = static_cast<FieldLineIntersections*>(&(Blackboard::getInstance()["FieldLineIntersections"]));

  if(theCameraInfo == nullptr || theCameraMatrix == nullptr || theImageCoordinateSystem == nullptr || theFieldLineIntersections == nullptr)
    return;

  const FieldLineIntersections::Intersection& intersection = theFieldLineIntersections->intersections[intersectionIndex];

  DEBUG_DRAWING("representation:MarkedField:field", "drawingOnField")
  {
    switch(intersection.type)
    {
      case FieldLineIntersections::Intersection::X:
        LINE("representation:MarkedField:field", intersection.pos.x() - intersection.dir1.x() * 100, intersection.pos.y() - intersection.dir1.y() * 100,
             intersection.pos.x() + intersection.dir1.x() * 100, intersection.pos.y() + intersection.dir1.y() * 100,
             15, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:MarkedField:field", intersection.pos.x() - intersection.dir2.x() * 100, intersection.pos.y() - intersection.dir2.y() * 100,
             intersection.pos.x() + intersection.dir2.x() * 100, intersection.pos.y() + intersection.dir2.y() * 100,
             15, Drawings::solidPen, ColorRGBA::blue);
        break;
      case FieldLineIntersections::Intersection::T:
        LINE("representation:MarkedField:field", intersection.pos.x(), intersection.pos.y(),
             intersection.pos.x() + intersection.dir1.x() * 100, intersection.pos.y() + intersection.dir1.y() * 100,
             15, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:MarkedField:field", intersection.pos.x() - intersection.dir2.x() * 100, intersection.pos.y() - intersection.dir2.y() * 100,
             intersection.pos.x() + intersection.dir2.x() * 100, intersection.pos.y() + intersection.dir2.y() * 100,
             15, Drawings::solidPen, ColorRGBA::blue);
        break;
      case FieldLineIntersections::Intersection::L:
        LINE("representation:MarkedField:field", intersection.pos.x(), intersection.pos.y(),
             intersection.pos.x() + intersection.dir1.x() * 100, intersection.pos.y() + intersection.dir1.y() * 100,
             15, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:MarkedField:field", intersection.pos.x(), intersection.pos.y(),
             intersection.pos.x() + intersection.dir2.x() * 100, intersection.pos.y() + intersection.dir2.y() * 100,
             15, Drawings::solidPen, ColorRGBA::blue);
        break;
    }
  }

  DEBUG_DRAWING("representation:MarkedField:image", "drawingOnImage")
  {
    Vector2f p1 = Vector2f::Zero();
    Vector2f p2 = Vector2f::Zero();
    Vector2f p3 = Vector2f::Zero();
    Vector2f p4 = Vector2f::Zero();
    const Vector2f dir1 = intersection.dir1 * 100.f;
    const Vector2f dir2 = intersection.dir2 * 100.f;
    bool transformationSuccessful = true;
    switch(intersection.type)
    {
      case FieldLineIntersections::Intersection::X:
        transformationSuccessful = Transformation::robotToImage(Vector2f(intersection.pos - dir1), *theCameraMatrix, *theCameraInfo, p1) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos + dir1), *theCameraMatrix, *theCameraInfo, p2) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos - dir2), *theCameraMatrix, *theCameraInfo, p3) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos + dir2), *theCameraMatrix, *theCameraInfo, p4);
        break;
      case FieldLineIntersections::Intersection::T:
        transformationSuccessful = Transformation::robotToImage(Vector2f(intersection.pos), *theCameraMatrix, *theCameraInfo, p1) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos + dir1), *theCameraMatrix, *theCameraInfo, p2) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos - dir2), *theCameraMatrix, *theCameraInfo, p3) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos + dir2), *theCameraMatrix, *theCameraInfo, p4);
        break;
      case FieldLineIntersections::Intersection::L:
        transformationSuccessful = Transformation::robotToImage(Vector2f(intersection.pos), *theCameraMatrix, *theCameraInfo, p1) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos + dir1), *theCameraMatrix, *theCameraInfo, p2) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos), *theCameraMatrix, *theCameraInfo, p3) &&
                                   Transformation::robotToImage(Vector2f(intersection.pos + dir2), *theCameraMatrix, *theCameraInfo, p4);
        break;
    }
    if(transformationSuccessful)
    {
      const Vector2f uncor1 = theImageCoordinateSystem->fromCorrected(p1);
      const Vector2f uncor2 = theImageCoordinateSystem->fromCorrected(p2);
      const Vector2f uncor3 = theImageCoordinateSystem->fromCorrected(p3);
      const Vector2f uncor4 = theImageCoordinateSystem->fromCorrected(p4);
      ARROW("representation:FieldLines:image", uncor1.x(), uncor1.y(), uncor2.x(), uncor2.y(),
            3, Drawings::solidPen, ColorRGBA::blue);
      ARROW("representation:FieldLines:image", uncor3.x(), uncor3.y(), uncor4.x(), uncor4.y(),
            3, Drawings::solidPen, ColorRGBA::green);
      Vector2f intersectionInImage;
      if(Transformation::robotToImage(intersection.pos, *theCameraMatrix, *theCameraInfo, intersectionInImage))
      {
        const Vector2f uncorIntersection = theImageCoordinateSystem->fromCorrected(intersectionInImage);
        DRAWTEXT("representation:FieldLines:image", uncorIntersection.x(), uncorIntersection.y(), 25, ColorRGBA(255, 180, 180),
                 (intersection.type == FieldLineIntersections::Intersection::L ? "L" : intersection.type == FieldLineIntersections::Intersection::T ? "T" : "X") <<
                 (intersection.additionalType == FieldLineIntersections::Intersection::none ? "" : intersection.additionalType == FieldLineIntersections::Intersection::mid ? "m" : "b"));
      }
    }
  }

  DECLARE_DEBUG_DRAWING3D("representation:MarkedField", "robot");
  TRANSLATE3D("representation:MarkedField", 0, 0, -210);
  COMPLEX_DRAWING3D("representation:MarkedField")
  {
    switch(intersection.type)
    {
      case FieldLineIntersections::Intersection::X:
        LINE3D("representation:MarkedField", intersection.pos.x() - intersection.dir1.x() * 100, intersection.pos.y() - intersection.dir1.y() * 100, 0,
               intersection.pos.x() + intersection.dir1.x() * 100, intersection.pos.y() + intersection.dir1.y() * 100, 0,
               2, ColorRGBA::blue);
        LINE3D("representation:MarkedField", intersection.pos.x() - intersection.dir2.x() * 100, intersection.pos.y() - intersection.dir2.y() * 100, 0,
               intersection.pos.x() + intersection.dir2.x() * 100, intersection.pos.y() + intersection.dir2.y() * 100, 0,
               2, ColorRGBA::blue);
        break;
      case FieldLineIntersections::Intersection::T:
        LINE3D("representation:MarkedField", intersection.pos.x(), intersection.pos.y(), 0,
               intersection.pos.x() + intersection.dir1.x() * 100, intersection.pos.y() + intersection.dir1.y() * 100, 0,
               2, ColorRGBA::blue);
        LINE3D("representation:MarkedField", intersection.pos.x() - intersection.dir2.x() * 100, intersection.pos.y() - intersection.dir2.y() * 100, 0,
               intersection.pos.x() + intersection.dir2.x() * 100, intersection.pos.y() + intersection.dir2.y() * 100, 0,
               2, ColorRGBA::blue);
        break;
      case FieldLineIntersections::Intersection::L:
        LINE3D("representation:MarkedField", intersection.pos.x(), intersection.pos.y(), 0,
               intersection.pos.x() + intersection.dir1.x() * 100, intersection.pos.y() + intersection.dir1.y() * 100, 0,
               2, ColorRGBA::blue);
        LINE3D("representation:MarkedField", intersection.pos.x(), intersection.pos.y(), 0,
               intersection.pos.x() + intersection.dir2.x() * 100, intersection.pos.y() + intersection.dir2.y() * 100, 0,
               2, ColorRGBA::blue);
        break;
    }
  }
}

MarkedIntersection::IntersectionMarker MarkedIntersection::mirror(const MarkedIntersection::IntersectionMarker& marker)
{
  return IntersectionMarker(marker + (marker < firstIntersectionMarkerOther ? firstIntersectionMarkerOther : -firstIntersectionMarkerOther));
};

MarkedIntersection::IntersectionMarker MarkedIntersection::mirror() const
{
  return mirror(marker);
};

void MarkedPoint::draw() const
{
  //TODO
}

MarkedPoint::PointMarker MarkedPoint::mirror(const MarkedPoint::PointMarker& marker)
{
  if(!marker)
    return marker;
  return PointMarker(marker + (marker < firstPointMarkerOther ? firstPointMarkerOther - 1 : -firstPointMarkerOther + 1));
};

MarkedPoint::PointMarker MarkedPoint::mirror() const
{
  return mirror(marker);
};
