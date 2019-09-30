/**
 * @file MidCircle.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "MidCircle.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"

void MidCircle::draw() const
{
  FieldFeature::draw();

  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    std::string thread = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::upper ? "Upper" : "Lower";
    DEBUG_DRAWING("representation:MidCircle:image", "drawingOnImage")
      THREAD("representation:MidCircle:image", thread);
    DEBUG_DRAWING("representation:MidCircle:field", "drawingOnField")
      THREAD("representation:MidCircle:field", thread);

    if(!isValid)
      return;

    COMPLEX_DRAWING("representation:MidCorner:field")
    {
      if(Blackboard::getInstance().exists("FieldDimensions") && Blackboard::getInstance().exists("CameraInfo"))
      {
        const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        CIRCLE("representation:MidCircle:field", this->translation.x(), this->translation.y(), theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
        Vector2f p1 = *this * Vector2f(0, theFieldDimensions.centerCircleRadius);
        Vector2f p2 = *this * Vector2f(0, -theFieldDimensions.centerCircleRadius);
        LINE("representation:MidCircle:field", p1.x(), p1.y(), p2.x(), p2.y(), 40, Drawings::solidPen, ColorRGBA::blue);
      }
    }

    COMPLEX_DRAWING("representation:MidCircle:image")
    {
      if(Blackboard::getInstance().exists("FieldDimensions") && Blackboard::getInstance().exists("CameraMatrix")
         && Blackboard::getInstance().exists("ImageCoordinateSystem"))
      {
        const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
        const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
        const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        const ImageCoordinateSystem& theImageCoordinateSystem = static_cast<const ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);

        Vector2f pInImage;
        if(Transformation::robotToImage(this->translation, theCameraMatrix, theCameraInfo, pInImage))
        {
          CROSS("representation:MidCircle:image", pInImage.x(), pInImage.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
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
            LINE("representation:MidCircle:image", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
          }
        }
        Vector2f p1;
        Vector2f p2;
        if(Transformation::robotToImage(*this * Vector2f(0, theFieldDimensions.centerCircleRadius), theCameraMatrix, theCameraInfo, p1) &&
           Transformation::robotToImage(*this * Vector2f(0, -theFieldDimensions.centerCircleRadius), theCameraMatrix, theCameraInfo, p2))
        {
          p1 = theImageCoordinateSystem.fromCorrected(p1);
          p2 = theImageCoordinateSystem.fromCorrected(p2);
          LINE("representation:MidCircle:image", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
        }
      }
    }
  }
}

const Pose2f MidCircle::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  return Pose2f(0.f, 0.f, 0.f);
}
