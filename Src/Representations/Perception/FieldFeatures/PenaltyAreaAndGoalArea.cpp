/**
 * @file PenaltyAreaAndGoalArea.cpp
 *
 * Implements a struct that represents the penalty area and goal area as a field feature.
 *
 * @author Nico Wellbrock
 * @author Tim Gebers
 */

#include "Debugging/DebugDrawings.h"
#include "PenaltyAreaAndGoalArea.h"
#include "Framework/Blackboard.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/FieldDimensions.h"

void PenaltyAreaAndGoalArea::draw() const
{
  FieldFeature::draw();
  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
    DEBUG_DRAWING("representation:PenaltyAreaAndGoalArea:field", "drawingOnField")
      THREAD("representation:PenaltyAreaAndGoalArea:field", theCameraInfo.getThreadName());
    DEBUG_DRAWING("representation:PenaltyAreaAndGoalArea:image", "drawingOnImage")
      THREAD("representation:PenaltyAreaAndGoalArea:image", theCameraInfo.getThreadName());

    if(!isValid)
      return;

    COMPLEX_DRAWING("representation:PenaltyAreaAndGoalArea:field")
    {
      ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      const float halfPenaltyAreaToGoalDistance = (theFieldDimensions.xPosOwnPenaltyArea - theFieldDimensions.xPosOwnGoalLine) / 2;
      const float halfPenaltyAreaWidth = (theFieldDimensions.yPosRightPenaltyArea - theFieldDimensions.yPosLeftPenaltyArea) / 2;
      Vector2f fieldCorner1 = (*this) * Vector2f(-halfPenaltyAreaToGoalDistance, -halfPenaltyAreaWidth);
      Vector2f fieldCorner2 = (*this) * Vector2f(-halfPenaltyAreaToGoalDistance, halfPenaltyAreaWidth);
      Vector2f fieldCorner3 = (*this) * Vector2f(halfPenaltyAreaToGoalDistance, halfPenaltyAreaWidth);
      Vector2f fieldCorner4 = (*this) * Vector2f(halfPenaltyAreaToGoalDistance, -halfPenaltyAreaWidth);

      LINE("representation:PenaltyAreaAndGoalArea:field", fieldCorner1.x(), fieldCorner1.y(), fieldCorner2.x(), fieldCorner2.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:PenaltyAreaAndGoalArea:field", fieldCorner2.x(), fieldCorner2.y(), fieldCorner3.x(), fieldCorner3.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:PenaltyAreaAndGoalArea:field", fieldCorner3.x(), fieldCorner3.y(), fieldCorner4.x(), fieldCorner4.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:PenaltyAreaAndGoalArea:field", fieldCorner4.x(), fieldCorner4.y(), fieldCorner1.x(), fieldCorner1.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      CROSS("representation:PenaltyAreaAndGoalArea:field", this->translation.x(), this->translation.y(), 40, 10, Drawings::solidPen, ColorRGBA::blue);
    }

    COMPLEX_DRAWING("representation:PenaltyAreaAndGoalArea:image")
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
          CROSS("representation:PenaltyAreaAndGoalArea:image", pInImage.x(), pInImage.y(), 5, 3, Drawings::solidPen, ColorRGBA::blue);
        }
        const float halfPenaltyAreaToGoalDistance = (theFieldDimensions.xPosOwnPenaltyArea - theFieldDimensions.xPosOwnGoalLine) / 2;
        const float halfPenaltyAreaWidth = (theFieldDimensions.yPosRightPenaltyArea - theFieldDimensions.yPosLeftPenaltyArea) / 2;
        Vector2f fieldCorner1 = (*this) * Vector2f(-halfPenaltyAreaToGoalDistance, -halfPenaltyAreaWidth);
        Vector2f fieldCorner2 = (*this) * Vector2f(-halfPenaltyAreaToGoalDistance, halfPenaltyAreaWidth);
        Vector2f fieldCorner3 = (*this) * Vector2f(halfPenaltyAreaToGoalDistance, halfPenaltyAreaWidth);
        Vector2f fieldCorner4 = (*this) * Vector2f(halfPenaltyAreaToGoalDistance, -halfPenaltyAreaWidth);
        Vector2f p1;
        Vector2f p2;
        Vector2f p3;
        Vector2f p4;

        bool tp1 = Transformation::robotToImage(fieldCorner1, theCameraMatrix, theCameraInfo, p1);
        if(tp1)
          p1 = theImageCoordinateSystem.fromCorrected(p1);
        bool tp2 = Transformation::robotToImage(fieldCorner2, theCameraMatrix, theCameraInfo, p2);
        if(tp2)
          p2 = theImageCoordinateSystem.fromCorrected(p2);
        bool tp3 = Transformation::robotToImage(fieldCorner3, theCameraMatrix, theCameraInfo, p3);
        if(tp3)
          p3 = theImageCoordinateSystem.fromCorrected(p3);
        bool tp4 = Transformation::robotToImage(fieldCorner4, theCameraMatrix, theCameraInfo, p4);
        if(tp4)
          p4 = theImageCoordinateSystem.fromCorrected(p4);

        if(tp1 && tp2)
          LINE("representation:PenaltyAreaAndGoalArea:image", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
        if(tp2 && tp3)
          LINE("representation:PenaltyAreaAndGoalArea:image", p2.x(), p2.y(), p3.x(), p3.y(), 3, Drawings::solidPen, ColorRGBA::blue);
        if(tp3 && tp4)
          LINE("representation:PenaltyAreaAndGoalArea:image", p3.x(), p3.y(), p4.x(), p4.y(), 3, Drawings::solidPen, ColorRGBA::blue);
        if(tp4 && tp1)
          LINE("representation:PenaltyAreaAndGoalArea:image", p4.x(), p4.y(), p1.x(), p1.y(), 3, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}

const Pose2f PenaltyAreaAndGoalArea::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
  const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
  return Pose2f(0.f, (theFieldDimensions.xPosOpponentPenaltyArea + theFieldDimensions.xPosOpponentGoalLine) / 2, 0.f);
}
