/**
 * @file RealisticBallPercepts.cpp
 *
 * Representation of the balls seen in the current frame
 *
 * @author Felix Thielke
 */

#include "RealisticBallPercepts.h"
#include "Tools/Debugging/DebugDrawings.h"

void RealisticBallPercepts::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RealisticBallPercepts:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:RealisticBallPercepts:Field", "drawingOnField");
  for(const RealisticBallPercept& ball : balls)
  {
    CIRCLE("representation:RealisticBallPercepts:Image", ball.positionInImage.x(), ball.positionInImage.y(), ball.radiusInImage, 2, Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
    CIRCLE("representation:RealisticBallPercepts:Field", ball.relativePositionOnField.x(), ball.relativePositionOnField.y(), ball.radiusOnField, 0, Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::red);
  }
}
