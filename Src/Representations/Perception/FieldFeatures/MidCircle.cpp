/**
 * @file MidCircle.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "../ImagePreprocessing/CameraMatrix.h"
#include "MidCircle.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"

void MidCircle::draw() const
{
  FieldFeature::draw();
  DECLARE_DEBUG_DRAWING("representation:MidCircle:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:MidCircle:field", "drawingOnField");
  if(!isValid)
    return;

  COMPLEX_DRAWING("representation:MidCorner:field")
  {
    ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
    const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
    CIRCLE("representation:MidCircle:field", this->translation.x(), this->translation.y(), theFieldDimensions.centerCircleRadius, 40, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
  }
  COMPLEX_DRAWING("representation:MidCircle:image")
  {
    if(Blackboard::getInstance().exists("CameraMatrix") && Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
      const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);

      ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);

      Vector2f pInImage;
      if(Transformation::robotToImage(this->translation, theCameraMatrix, theCameraInfo, pInImage))
      {
        CROSS("representation:MidCircle:image", pInImage.x(), pInImage.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
      }
      const float stepSize = 0.2f;
      for(float i = 0; i < pi2; i += stepSize)
      {
        Vector2f p1 = Vector2f::Zero();
        Vector2f p2 = Vector2f::Zero();
        if(Transformation::robotToImage(Vector2f(this->translation + Vector2f(theFieldDimensions.centerCircleRadius, 0).rotate(i)), theCameraMatrix, theCameraInfo, p1) &&
           Transformation::robotToImage(Vector2f(this->translation + Vector2f(theFieldDimensions.centerCircleRadius, 0).rotate(i + stepSize)), theCameraMatrix, theCameraInfo, p2))
          LINE("representation:MidCircle:image", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}

const Pose2f MidCircle::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  return Pose2f(0.f, 0.f, 0.f);
}
