/**
 * @file GoalFeature.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "../ImagePreprocessing/CameraMatrix.h"
#include "GoalFeature.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"

void GoalFeature::draw() const
{
  FieldFeature::draw();
  DECLARE_DEBUG_DRAWING("representation:GoalFeature:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalFeature:field", "drawingOnField");
  if(!isValid)
    return;

  static const float size = 1000.;
  COMPLEX_DRAWING("representation:GoalFeature:field")
  {
    const Vector2f a = (*this) * Vector2f(0.f, size);
    const Vector2f b = (*this) * Vector2f(0.f, -size);
    const Vector2f c = (*this) * Vector2f(size, 0.f);
    LINE("representation:GoalFeature:field", a.x(), a.y(), b.x(), b.y(), 10, Drawings::solidPen, ColorRGBA::blue);
    LINE("representation:GoalFeature:field", a.x(), a.y(), c.x(), c.y(), 10, Drawings::solidPen, ColorRGBA::blue);
    LINE("representation:GoalFeature:field", b.x(), b.y(), c.x(), c.y(), 10, Drawings::solidPen, ColorRGBA::blue);
    //DRAWTEXT("representation:GoalFeature:field", this->translation.x(), this->translation.y(), 40, ColorRGBA::blue, "TG");
  }
  COMPLEX_DRAWING("representation:GoalFeature:image")
  {
    if(Blackboard::getInstance().exists("CameraMatrix") && Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
      const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);

      const Vector2f a = (*this) * Vector2f(0.f, size);
      const Vector2f b = (*this) * Vector2f(0.f, -size);
      const Vector2f c = (*this) * Vector2f(size, 0.f);
      Vector2f aImage, bImage, cImage;

      if(Transformation::robotToImage(a, theCameraMatrix, theCameraInfo, aImage) &&
         Transformation::robotToImage(b, theCameraMatrix, theCameraInfo, bImage) &&
         Transformation::robotToImage(c, theCameraMatrix, theCameraInfo, cImage))
      {
        LINE("representation:GoalFeature:image", aImage.x(), aImage.y(), bImage.x(), bImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:GoalFeature:image", aImage.x(), aImage.y(), cImage.x(), cImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:GoalFeature:image", bImage.x(), bImage.y(), cImage.x(), cImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);

        LINE("representation:GoalFeature:image", bImage.x(), bImage.y(), bImage.x(), 0, 2, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:GoalFeature:image", aImage.x(), aImage.y(), aImage.x(), 0, 2, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}

const Pose2f GoalFeature::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
  const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);

  return Pose2f(theFieldDimensions.xPosOpponentGroundline, 0.f);
}
