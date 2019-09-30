/**
 * @file GoalFrame.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "GoalFrame.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"

void GoalFrame::draw() const
{
  FieldFeature::draw();

  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    std::string thread = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::upper ? "Upper" : "Lower";
    DEBUG_DRAWING("representation:GoalFrame:image", "drawingOnImage")
      THREAD("representation:GoalFrame:image", thread);
    DEBUG_DRAWING("representation:GoalFrame:field", "drawingOnField")
      THREAD("representation:GoalFrame:field", thread);

    if(!isValid)
      return;

    static const float size = 1000.f;
    COMPLEX_DRAWING("representation:GoalFrame:field")
    {
      const Vector2f a = (*this) * Vector2f(0.f, size);
      const Vector2f b = (*this) * Vector2f(0.f, -size);
      const Vector2f c = (*this) * Vector2f(size, 0.f);
      const Vector2f d = (*this) * Vector2f(size, size);
      const Vector2f e = (*this) * Vector2f(size, -size);
      LINE("representation:GoalFrame:field", a.x(), a.y(), b.x(), b.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:GoalFrame:field", a.x(), a.y(), c.x(), c.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:GoalFrame:field", b.x(), b.y(), c.x(), c.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:GoalFrame:field", d.x(), d.y(), e.x(), e.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:GoalFrame:field", b.x(), b.y(), e.x(), e.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:GoalFrame:field", d.x(), d.y(), a.x(), a.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      DRAWTEXT("representation:GoalFrame:field", this->translation.x(), this->translation.y(), 40, ColorRGBA::blue, "GF");
    }

    DEBUG_DRAWING("representation:GoalFrame:image", "drawingOnImage")
    {
      if(Blackboard::getInstance().exists("CameraMatrix") && Blackboard::getInstance().exists("ImageCoordinateSystem"))
      {
        const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
        const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
        const ImageCoordinateSystem& theImageCoordinateSystem = static_cast<const ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);
        const Vector2f a = (*this) * Vector2f(0.f, size);
        const Vector2f b = (*this) * Vector2f(0.f, -size);
        const Vector2f c = (*this) * Vector2f(size, 0.f);
        const Vector2f d = (*this) * Vector2f(size, size);
        const Vector2f e = (*this) * Vector2f(size, -size);
        Vector2f aImage, bImage, cImage, dImage, eImage;

        if(Transformation::robotToImage(a, theCameraMatrix, theCameraInfo, aImage) &&
           Transformation::robotToImage(b, theCameraMatrix, theCameraInfo, bImage) &&
           Transformation::robotToImage(c, theCameraMatrix, theCameraInfo, cImage) &&
           Transformation::robotToImage(d, theCameraMatrix, theCameraInfo, dImage) &&
           Transformation::robotToImage(e, theCameraMatrix, theCameraInfo, eImage))
        {
          aImage = theImageCoordinateSystem.fromCorrected(aImage);
          bImage = theImageCoordinateSystem.fromCorrected(bImage);
          cImage = theImageCoordinateSystem.fromCorrected(cImage);
          dImage = theImageCoordinateSystem.fromCorrected(dImage);
          eImage = theImageCoordinateSystem.fromCorrected(eImage);
          LINE("representation:GoalFrame:image", aImage.x(), aImage.y(), bImage.x(), bImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:GoalFrame:image", aImage.x(), aImage.y(), cImage.x(), cImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:GoalFrame:image", bImage.x(), bImage.y(), cImage.x(), cImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:GoalFrame:image", dImage.x(), dImage.y(), eImage.x(), eImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:GoalFrame:image", bImage.x(), bImage.y(), eImage.x(), eImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:GoalFrame:image", dImage.x(), dImage.y(), aImage.x(), aImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
        }
      }
    }
  }
}

const Pose2f GoalFrame::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
  const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);

  return Pose2f(theFieldDimensions.xPosOpponentGroundline, 0.f);
}
