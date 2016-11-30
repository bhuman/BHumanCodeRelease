#include "OdometryAnalyzation.h"

void OdometryAnalyzation::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:OdometryAnalyzation:draw", "drawingOnField");
  DEBUG_DRAWING("representation:OdometryAnalyzation:draw", "drawingOnField")
  {
    LINE("representation:OdometryAnalyzation:draw", startPoint.translation.x(), startPoint.translation.y(),
      currentEndPoint.translation.x(), currentEndPoint.translation.y(), 100, Drawings::solidPen, ColorRGBA::red);
    COVARIANCE2D("representation:OdometryAnalyzation:draw", covariance, currentEndPoint.translation);
  }
}
