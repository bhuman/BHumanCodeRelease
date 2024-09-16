/**
* @file PenaltyMarkWithPenaltyAreaLine.h
*
* Implementation of a struct that represents the combination of
* a penalty mark and the line of the penalty area that is
* closest to it (the line between penalty mark and the field's
* halfway line):
*
*                  (goal)
*-----------------------------------------
*          |                   |
*          |         + (mark)  |
*          ===================== (<- this line)
*
* @author Tim Laue
*/

#include "PenaltyMarkWithPenaltyAreaLine.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Blackboard.h"

// TODO: Drawing in image

void PenaltyMarkWithPenaltyAreaLine::draw() const
{
  FieldFeature::draw();
  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
    DEBUG_DRAWING("representation:PenaltyMarkWithPenaltyAreaLine:field", "drawingOnField")
      THREAD("representation:PenaltyMarkWithPenaltyAreaLine:field", theCameraInfo.getThreadName());

    if(!isValid)
      return;

    static const float size = 1000.f;
    COMPLEX_DRAWING("representation:PenaltyMarkWithPenaltyAreaLine:field")
    {
      ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      const float distLinePenaltyMark = theFieldDimensions.xPosOpponentPenaltyMark - theFieldDimensions.xPosOpponentPenaltyArea;
      const Vector2f pos = this->translation;
      const Vector2f lineL = (*this) * Vector2f(0.f, size/2.f);
      const Vector2f lineR = (*this) * Vector2f(0.f, -size/2.f);
      const Vector2f arrowHead = (*this) * Vector2f(size, 0.f);
      const Vector2f triangleL = (*this) * Vector2f(0.f, size/4.f);
      const Vector2f triangleR = (*this) * Vector2f(0.f, -size/4.f);
      const Vector2f triangleHead = (*this) * Vector2f(distLinePenaltyMark, 0.f);
      // Base line:
      LINE("representation:PenaltyMarkWithPenaltyAreaLine:field", lineL.x(), lineL.y(), lineR.x(), lineR.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      // Arrow towards goal center:
      ARROW("representation:PenaltyMarkWithPenaltyAreaLine:field", pos.x(), pos.y(), arrowHead.x(), arrowHead.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      // Small triangle from base line to penalty mark:
      LINE("representation:PenaltyMarkWithPenaltyAreaLine:field", triangleL.x(), triangleL.y(), triangleHead.x(), triangleHead.y(), 10, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:PenaltyMarkWithPenaltyAreaLine:field", triangleR.x(), triangleR.y(), triangleHead.x(), triangleHead.y(), 10, Drawings::solidPen, ColorRGBA::blue);
    }
  }
}

const Pose2f PenaltyMarkWithPenaltyAreaLine::getGlobalFeaturePosition() const
{
  ASSERT(isValid);
  ASSERT(Blackboard::getInstance().exists("FieldDimensions"));
  const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
  return Pose2f(0.f, theFieldDimensions.xPosOpponentPenaltyArea, 0.f);
}
