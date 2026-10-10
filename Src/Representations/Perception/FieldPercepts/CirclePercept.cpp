/**
 * @file CirclePercept.cpp
 * Implementation of a struct that represents the center circle.
 * @author Jesse Richter-Klug
 */

#include "CirclePercept.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"
#include "Framework/Blackboard.h"

void CirclePercept::draw() const
{
  FieldDimensions* theFieldDimensions = nullptr;
  CameraInfo* theCameraInfo = nullptr;
  CameraMatrix* theCameraMatrix = nullptr;

  if(Blackboard::getInstance().exists("FieldDimensions"))
    theFieldDimensions = static_cast<FieldDimensions*>(&(Blackboard::getInstance()["FieldDimensions"]));
  if(Blackboard::getInstance().exists("CameraInfo"))
    theCameraInfo = static_cast<CameraInfo*>(&(Blackboard::getInstance()["CameraInfo"]));
  if(Blackboard::getInstance().exists("CameraMatrix"))
    theCameraMatrix = static_cast<CameraMatrix*>(&(Blackboard::getInstance()["CameraMatrix"]));

  if(theFieldDimensions == nullptr || theCameraInfo == nullptr || theCameraMatrix == nullptr)
    return;

  DEBUG_DRAWING("representation:CirclePercept:field", "drawingOnField")
  {
    if(wasSeen)
    {
      CROSS("representation:CirclePercept:field", pos.x(), pos.y(), 40, 40, Drawings::solidPen, ColorRGBA::black);
      CIRCLE("representation:CirclePercept:field", pos.x(), pos.y(), theFieldDimensions->centerCircleRadius, 30, Drawings::solidPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA::black);
    }
  }

  DEBUG_DRAWING("representation:FieldLines:field", "drawingOnField")
  {
    if(wasSeen)
    {
      CROSS("representation:FieldLines:field", pos.x(), pos.y(), 40, 40, Drawings::solidPen, ColorRGBA::red);
      CIRCLE("representation:FieldLines:field", pos.x(), pos.y(), theFieldDimensions->centerCircleRadius, 30, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
    }
  }

  DEBUG_DRAWING("representation:FieldLines:image", "drawingOnImage")
  {
    if(wasSeen)
    {
      Vector2f p1;
      if(Transformation::robotToImage(pos, *theCameraMatrix, *theCameraInfo, p1))
        CROSS("representation:FieldLines:image", p1.x(), p1.y(), 5, 3, Drawings::solidPen, ColorRGBA::red);
      const float stepSize = 0.2f;
      for(float i = 0; i < pi2; i += stepSize)
      {
        Vector2f p1 = Vector2f::Zero();
        Vector2f p2 = Vector2f::Zero();
        if(Transformation::robotToImage(Vector2f(pos + Vector2f(theFieldDimensions->centerCircleRadius, 0).rotate(i)), *theCameraMatrix, *theCameraInfo, p1) &&
           Transformation::robotToImage(Vector2f(pos + Vector2f(theFieldDimensions->centerCircleRadius, 0).rotate(i + stepSize)), *theCameraMatrix, *theCameraInfo, p2))
          LINE("representation:FieldLines:image", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::red);
      }
    }
  }

  DECLARE_DEBUG_DRAWING3D("representation:FieldLines", "robot");
  TRANSLATE3D("representation:FieldLines", 0, 0, -210);
  COMPLEX_DRAWING3D("representation:FieldLines")
  {
    if(wasSeen)
    {
      Vector2f v1(pos.x() + theFieldDimensions->centerCircleRadius, pos.y());
      for(int i = 1; i < 33; ++i)
      {
        const float angle = i * pi2 / 32;
        const Vector2f v2(pos.x() + std::cos(angle) * (theFieldDimensions->centerCircleRadius),
                          pos.y() + std::sin(angle) * (theFieldDimensions->centerCircleRadius));
        LINE3D("representation:FieldLines", v1.x(), v1.y(), 0, v2.x(), v2.y(), 0, 2, ColorRGBA::red);
        v1 = v2;
      }
    }
  }
}

void CirclePercept::verify() const
{
  if(wasSeen)
  {
    ASSERT(std::isfinite(pos.x()));
    ASSERT(std::isfinite(pos.y()));
    ASSERT(std::isnormal(cov(0, 0)));
    ASSERT(std::isnormal(cov(1, 1)));
    ASSERT(std::isfinite(cov(0, 1)));
    ASSERT(std::isfinite(cov(1, 0)));
  }
}
