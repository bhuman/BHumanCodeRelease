/**
* @file CenterCircleWithLine.cpp
*
* Implementation of a struct that represents the combination of
* the center circle and the halfway line crossing the circle:
*
*                   _____
*                  *     *      (ugly, I know)
*                 /       \
*    ------------+----+----+------------
*                 \       /
*                  *     *
*                   -----
*
* @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
* @author Tim Laue
*/

#include "CenterCircleWithLine.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Blackboard.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"

void CenterCircleWithLine::draw() const
{
  FieldFeature::draw();

  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
    DEBUG_DRAWING("representation:CenterCircleWithLine:image", "drawingOnImage")
      THREAD("representation:CenterCircleWithLine:image", theCameraInfo.getThreadName());
    DEBUG_DRAWING("representation:CenterCircleWithLine:field", "drawingOnField")
      THREAD("representation:CenterCircleWithLine:field", theCameraInfo.getThreadName());

    if(!isValid)
      return;

    COMPLEX_DRAWING("representation:CenterCircleWithLine:field")
    {
      if(Blackboard::getInstance().exists("FieldDimensions"))
      {
        //TODO: Set color back to blue later.
        const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        CIRCLE("representation:CenterCircleWithLine:field", this->translation.x(), this->translation.y(), theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
        Vector2f p1 = *this * Vector2f(0, theFieldDimensions.centerCircleRadius + 200.f);
        Vector2f p2 = *this * Vector2f(0, -theFieldDimensions.centerCircleRadius - 200.f);
        LINE("representation:CenterCircleWithLine:field", p1.x(), p1.y(), p2.x(), p2.y(), 40, Drawings::solidPen, ColorRGBA::blue);
      }
    }

    COMPLEX_DRAWING("representation:CenterCircleWithLine:image")
    {
      if(Blackboard::getInstance().exists("FieldDimensions") && Blackboard::getInstance().exists("CameraMatrix")
         && Blackboard::getInstance().exists("ImageCoordinateSystem"))
      {
        const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
        const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        const ImageCoordinateSystem& theImageCoordinateSystem = static_cast<const ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);

        Vector2f pInImage;
        if(Transformation::robotToImage(this->translation, theCameraMatrix, theCameraInfo, pInImage))
        {
          CROSS("representation:CenterCircleWithLine:image", pInImage.x(), pInImage.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
        }
        const float stepSize = 0.2f;
        for(float i = 0; i < pi2; i += stepSize)
        {
          Vector2f p1;
          Vector2f p2;
          if(Transformation::robotToImage(Vector2f(this->translation + Vector2f(theFieldDimensions.centerCircleRadius, 0).rotate(i)), theCameraMatrix, theCameraInfo, p1) &&
             Transformation::robotToImage(Vector2f(this->translation + Vector2f(theFieldDimensions.centerCircleRadius, 0).rotate(i + stepSize)), theCameraMatrix, theCameraInfo, p2))
          {
            p1 = theImageCoordinateSystem.fromCorrected(p1);
            p2 = theImageCoordinateSystem.fromCorrected(p2);
            LINE("representation:CenterCircleWithLine:image", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
          }
        }
        Vector2f p1;
        Vector2f p2;
        if(Transformation::robotToImage(*this * Vector2f(0, theFieldDimensions.centerCircleRadius + 200.f), theCameraMatrix, theCameraInfo, p1) &&
           Transformation::robotToImage(*this * Vector2f(0, -theFieldDimensions.centerCircleRadius - 200.f), theCameraMatrix, theCameraInfo, p2))
        {
          p1 = theImageCoordinateSystem.fromCorrected(p1);
          p2 = theImageCoordinateSystem.fromCorrected(p2);
          LINE("representation:CenterCircleWithLine:image", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
        }
      }
    }
  }
}

const Pose2f CenterCircleWithLine::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  return Pose2f(0.f, 0.f, 0.f);
}
