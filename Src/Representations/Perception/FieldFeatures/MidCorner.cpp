/**
 * @file MidCorner.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "MidCorner.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"

void MidCorner::draw() const
{
  FieldFeature::draw();

  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    std::string thread = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::upper ? "Upper" : "Lower";
    DEBUG_DRAWING("representation:MidCorner:image", "drawingOnImage")
      THREAD("representation:MidCorner:image", thread);
    DEBUG_DRAWING("representation:MidCorner:field", "drawingOnField")
      THREAD("representation:MidCorner:field", thread);

    if(!isValid)
      return;

    static const float size = 1000.f;
    COMPLEX_DRAWING("representation:MidCorner:field")
    {
      const Vector2f a = (*this) * Vector2f(0.f, size);
      const Vector2f b = (*this) * Vector2f(0.f, -size);
      const Vector2f c = (*this) * Vector2f(size, 0.f);
      LINE("representation:MidCorner:field", a.x(), a.y(), b.x(), b.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:MidCorner:field", a.x(), a.y(), c.x(), c.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:MidCorner:field", b.x(), b.y(), c.x(), c.y(), 10, Drawings::solidPen, ColorRGBA::blue);
    }
    COMPLEX_DRAWING("representation:MidCorner:image")
    {
      if(Blackboard::getInstance().exists("CameraMatrix") && Blackboard::getInstance().exists("ImageCoordinateSystem"))
      {
        const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
        const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
        const ImageCoordinateSystem& theImageCoordinateSystem = static_cast<const ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);
        const Vector2f a = (*this) * Vector2f(0.f, size);
        const Vector2f b = (*this) * Vector2f(0.f, -size);
        const Vector2f c = (*this) * Vector2f(size, 0.f);
        Vector2f aImage, bImage, cImage;

        if(Transformation::robotToImage(a, theCameraMatrix, theCameraInfo, aImage) &&
           Transformation::robotToImage(b, theCameraMatrix, theCameraInfo, bImage) &&
           Transformation::robotToImage(c, theCameraMatrix, theCameraInfo, cImage))
        {
          aImage = theImageCoordinateSystem.fromCorrected(aImage);
          bImage = theImageCoordinateSystem.fromCorrected(bImage);
          cImage = theImageCoordinateSystem.fromCorrected(cImage);
          LINE("representation:MidCorner:image", aImage.x(), aImage.y(), bImage.x(), bImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:MidCorner:image", aImage.x(), aImage.y(), cImage.x(), cImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:MidCorner:image", bImage.x(), bImage.y(), cImage.x(), cImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
        }
      }
    }
  }
}

const Pose2f MidCorner::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
  const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);

  const Vector2f dirPoint = (*this) * Vector2f(1.f, 0.f);

  bool isRightMidCorner(((dirPoint.x() - this->translation.x()) * (0.f - this->translation.y()) - (dirPoint.y() - this->translation.y()) * (0.f - this->translation.x())) > 0.f);
  return Pose2f(isRightMidCorner ? pi_2 : -pi_2, 0.f, isRightMidCorner ? theFieldDimensions.yPosRightSideline : theFieldDimensions.yPosLeftSideline);
}
