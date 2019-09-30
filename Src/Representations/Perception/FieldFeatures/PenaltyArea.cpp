/**
 * @file PenaltyArea.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "PenaltyArea.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Transformation.h"

void PenaltyArea::draw() const
{
  FieldFeature::draw();

  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    std::string thread = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::upper ? "Upper" : "Lower";
    DEBUG_DRAWING("representation:PenaltyArea:image", "drawingOnImage")
      THREAD("representation:PenaltyArea:field", thread);
    DEBUG_DRAWING("representation:PenaltyArea:field", "drawingOnField")
      THREAD("representation:PenaltyArea:image", thread);

    if(!isValid)
      return;

    COMPLEX_DRAWING("representation:PenaltyArea:field")
    {
      if(Blackboard::getInstance().exists("FieldDimensions"))
      {
        const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        const float halfLength = (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;
        const Vector2f leftBotton = (*this) * Vector2f(-halfLength, theFieldDimensions.yPosLeftPenaltyArea);
        const Vector2f leftTop = (*this) * Vector2f(halfLength, theFieldDimensions.yPosLeftPenaltyArea);
        const Vector2f rightBotton = (*this) * Vector2f(-halfLength, theFieldDimensions.yPosRightPenaltyArea);
        const Vector2f rightTop = (*this) * Vector2f(halfLength, theFieldDimensions.yPosRightPenaltyArea);
        const Vector2f midTop = (*this) * Vector2f(halfLength, 0.f);
        LINE("representation:PenaltyArea:field", leftBotton.x(), leftBotton.y(), rightBotton.x(), rightBotton.y(), 10, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:PenaltyArea:field", rightTop.x(), rightTop.y(), rightBotton.x(), rightBotton.y(), 10, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:PenaltyArea:field", leftBotton.x(), leftBotton.y(), leftTop.x(), leftTop.y(), 10, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:PenaltyArea:field", rightTop.x(), rightTop.y(), leftTop.x(), leftTop.y(), 10, Drawings::solidPen, ColorRGBA::blue);
        LINE("representation:PenaltyArea:field", midTop.x(), midTop.y(), this->translation.x(), this->translation.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      }
    }
    COMPLEX_DRAWING("representation:PenaltyArea:image")
    {
      if(Blackboard::getInstance().exists("FieldDimensions") && Blackboard::getInstance().exists("CameraMatrix")
         && Blackboard::getInstance().exists("ImageCoordinateSystem"))
      {
        const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
        const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
        const ImageCoordinateSystem& theImageCoordinateSystem = static_cast<const ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);

        const float halfLength = (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;
        const Vector2f leftBotton = (*this) * Vector2f(-halfLength, theFieldDimensions.yPosLeftPenaltyArea);
        const Vector2f leftTop = (*this) * Vector2f(halfLength, theFieldDimensions.yPosLeftPenaltyArea);
        const Vector2f rightBotton = (*this) * Vector2f(-halfLength, theFieldDimensions.yPosRightPenaltyArea);
        const Vector2f rightTop = (*this) * Vector2f(halfLength, theFieldDimensions.yPosRightPenaltyArea);
        const Vector2f midTop = (*this) * Vector2f(halfLength, 0.f);
        const Vector2f midMid = this->translation;
        Vector2f leftBottonInImage, leftTopInImage, rightBottonInImage, rightTopInImage, midTopInImage, midMidInImage;

        if(Transformation::robotToImage(leftBotton, theCameraMatrix, theCameraInfo, leftBottonInImage) &&
           Transformation::robotToImage(leftTop, theCameraMatrix, theCameraInfo, leftTopInImage) &&
           Transformation::robotToImage(rightTop, theCameraMatrix, theCameraInfo, rightTopInImage) &&
           Transformation::robotToImage(midTop, theCameraMatrix, theCameraInfo, midTopInImage) &&
           Transformation::robotToImage(rightBotton, theCameraMatrix, theCameraInfo, rightBottonInImage) &&
           Transformation::robotToImage(midMid, theCameraMatrix, theCameraInfo, midMidInImage))
        {
          leftBottonInImage = theImageCoordinateSystem.fromCorrected(leftBottonInImage);
          leftTopInImage = theImageCoordinateSystem.fromCorrected(leftTopInImage);
          rightTopInImage = theImageCoordinateSystem.fromCorrected(rightTopInImage);
          midTopInImage = theImageCoordinateSystem.fromCorrected(midTopInImage);
          rightBottonInImage = theImageCoordinateSystem.fromCorrected(rightBottonInImage);
          midMidInImage = theImageCoordinateSystem.fromCorrected(midMidInImage);
          LINE("representation:PenaltyArea:image", leftBottonInImage.x(), leftBottonInImage.y(), rightBottonInImage.x(), rightBottonInImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:PenaltyArea:image", rightTopInImage.x(), rightTopInImage.y(), rightBottonInImage.x(), rightBottonInImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:PenaltyArea:image", leftBottonInImage.x(), leftBottonInImage.y(), leftTopInImage.x(), leftTopInImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:PenaltyArea:image", rightTopInImage.x(), rightTopInImage.y(), leftTopInImage.x(), leftTopInImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
          LINE("representation:PenaltyArea:image", midTopInImage.x(), midTopInImage.y(), midMidInImage.x(), midMidInImage.y(), 2, Drawings::solidPen, ColorRGBA::blue);
        }
      }
    }
  }
}

const Pose2f PenaltyArea::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
  const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
  const float penaltyAreaDepth = (theFieldDimensions.xPosOpponentGroundline - theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;
  return Pose2f(0.f, theFieldDimensions.xPosOpponentGroundline - penaltyAreaDepth, 0.f);
}
