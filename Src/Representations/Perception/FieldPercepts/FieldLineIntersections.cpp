/**
 * @file FieldLineIntersections.cpp
 * Implementation of a struct that represents the fieldline intersections.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldLineIntersections.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"
#include "Framework/Blackboard.h"

void FieldLineIntersections::draw() const
{
  CameraInfo* theCameraInfo = nullptr;
  CameraMatrix* theCameraMatrix = nullptr;
  ImageCoordinateSystem* theImageCoordinateSystem = nullptr;

  if(Blackboard::getInstance().exists("CameraInfo"))
    theCameraInfo = static_cast<CameraInfo*>(&(Blackboard::getInstance()["CameraInfo"]));
  if(Blackboard::getInstance().exists("CameraMatrix"))
    theCameraMatrix = static_cast<CameraMatrix*>(&(Blackboard::getInstance()["CameraMatrix"]));
  if(Blackboard::getInstance().exists("ImageCoordinateSystem"))
    theImageCoordinateSystem =  static_cast<ImageCoordinateSystem*>(&(Blackboard::getInstance()["ImageCoordinateSystem"]));

  if(theCameraInfo == nullptr || theCameraMatrix == nullptr || theImageCoordinateSystem == nullptr)
    return;

  DEBUG_DRAWING("representation:FieldLines:field", "drawingOnField")
  {
    for(std::vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      switch(inter->type)
      {
        case Intersection::X:
          LINE("representation:FieldLines:field", inter->pos.x() - inter->dir1.x() * 100, inter->pos.y() - inter->dir1.y() * 100,
               inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100,
               15, Drawings::solidPen, ColorRGBA::red);
          LINE("representation:FieldLines:field", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100,
               inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100,
               15, Drawings::solidPen, ColorRGBA::red);
          break;
        case Intersection::T:
          LINE("representation:FieldLines:field", inter->pos.x(), inter->pos.y(),
               inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100,
               15, Drawings::solidPen, ColorRGBA::red);
          LINE("representation:FieldLines:field", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100,
               inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100,
               15, Drawings::solidPen, ColorRGBA::red);
          break;
        case Intersection::L:
          LINE("representation:FieldLines:field", inter->pos.x(), inter->pos.y(),
               inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100,
               15, Drawings::solidPen, ColorRGBA::red);
          LINE("representation:FieldLines:field", inter->pos.x(), inter->pos.y(),
               inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100,
               15, Drawings::solidPen, ColorRGBA::red);
          break;
      }
    }
  }

  DEBUG_DRAWING("representation:FieldLines:image", "drawingOnImage")
  {
    for(std::vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
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
        const Vector2f uncorrected1 = theImageCoordinateSystem->fromCorrected(p1);
        const Vector2f uncorrected2 = theImageCoordinateSystem->fromCorrected(p2);
        const Vector2f uncorrected3 = theImageCoordinateSystem->fromCorrected(p3);
        const Vector2f uncorrected4 = theImageCoordinateSystem->fromCorrected(p4);
        ARROW("representation:FieldLines:image", uncorrected1.x(), uncorrected1.y(), uncorrected2.x(), uncorrected2.y(),
              3, Drawings::solidPen, ColorRGBA::red);
        ARROW("representation:FieldLines:image", uncorrected3.x(), uncorrected3.y(), uncorrected4.x(), uncorrected4.y(),
              3, Drawings::solidPen, ColorRGBA::green);
        Vector2f intersectionInImage;
        if(Transformation::robotToImage(inter->pos, *theCameraMatrix, *theCameraInfo, intersectionInImage))
        {
          const Vector2f uncorrectedIntersection = theImageCoordinateSystem->fromCorrected(intersectionInImage);
          DRAW_TEXT("representation:FieldLines:image", uncorrectedIntersection.x(), uncorrectedIntersection.y(), 25, ColorRGBA(255, 180, 180),
                   (inter->type == Intersection::L ? "L" : inter->type == Intersection::T ? "T" : "X"));
        }
      }
    }
  }

  DECLARE_DEBUG_DRAWING3D("representation:FieldLines", "robot");
  TRANSLATE3D("representation:FieldLines", 0, 0, -210);
  COMPLEX_DRAWING3D("representation:FieldLines")
  {
    for(std::vector<Intersection>::const_iterator inter = intersections.begin(); inter != intersections.end(); inter++)
    {
      switch(inter->type)
      {
        case Intersection::X:
          LINE3D("representation:FieldLines", inter->pos.x() - inter->dir1.x() * 100, inter->pos.y() - inter->dir1.y() * 100, 0,
                 inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100, 0,
                 2, ColorRGBA::red);
          LINE3D("representation:FieldLines", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100, 0,
                 inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100, 0,
                 2, ColorRGBA::red);
          break;
        case Intersection::T:
          LINE3D("representation:FieldLines", inter->pos.x(), inter->pos.y(), 0,
                 inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100, 0,
                 2, ColorRGBA::red);
          LINE3D("representation:FieldLines", inter->pos.x() - inter->dir2.x() * 100, inter->pos.y() - inter->dir2.y() * 100, 0,
                 inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100, 0,
                 2, ColorRGBA::red);
          break;
        case Intersection::L:
          LINE3D("representation:FieldLines", inter->pos.x(), inter->pos.y(), 0,
                 inter->pos.x() + inter->dir1.x() * 100, inter->pos.y() + inter->dir1.y() * 100, 0,
                 2, ColorRGBA::red);
          LINE3D("representation:FieldLines", inter->pos.x(), inter->pos.y(), 0,
                 inter->pos.x() + inter->dir2.x() * 100, inter->pos.y() + inter->dir2.y() * 100, 0,
                 2, ColorRGBA::red);
          break;
      }
    }
  }
}

void FieldLineIntersections::verify() const
{
  for([[maybe_unused]] const Intersection& i : intersections)
  {
    ASSERT(std::isfinite(i.pos.x()));
    ASSERT(std::isfinite(i.pos.y()));
    ASSERT(std::isnormal(i.cov(0, 0)));
    ASSERT(std::isnormal(i.cov(1, 1)));
    ASSERT(std::isfinite(i.cov(0, 1)));
    ASSERT(std::isfinite(i.cov(1, 0)));
    ASSERT(std::isfinite(i.dir1.x()));
    ASSERT(std::isfinite(i.dir1.y()));
    ASSERT(std::isfinite(i.dir2.x()));
    ASSERT(std::isfinite(i.dir2.y()));
  }
}
