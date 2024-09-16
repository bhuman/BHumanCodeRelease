/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author Jesse
 */

#include "BallPercept.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"

void BallPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:covariance", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "robot");
  TRANSLATE3D("representation:BallPercept", 0, 0, -230);
  if(status == seen)
  {
    CIRCLE("representation:BallPercept:image", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
           Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 128, 64, 100));
    CIRCLE("representation:BallPercept:field", positionOnField.x(), positionOnField.y(), radiusOnField, 0, // pen width
           Drawings::solidPen, ColorRGBA::orange, Drawings::noBrush, ColorRGBA::orange);
    COVARIANCE_ELLIPSES_2D("representation:BallPercept:covariance", covarianceOnField, positionOnField);
    SPHERE3D("representation:BallPercept", positionOnField.x(), positionOnField.y(), radiusOnField, radiusOnField, ColorRGBA::orange);
  }
  else if(status == guessed)
    CIRCLE("representation:BallPercept:image", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
           Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(64, 128, 255, 90));
}

void BallPercept::verify() const
{
  if(status == seen || status == guessed)
  {
    ASSERT(std::isfinite(positionInImage.x()));
    ASSERT(std::isfinite(positionInImage.y()));
    ASSERT(std::isfinite(positionOnField.x()));
    ASSERT(std::isfinite(positionOnField.y()));
    ASSERT(std::isfinite(radiusInImage));
    ASSERT(std::isfinite(radiusOnField));
    ASSERT(radiusInImage > 0.f);
    ASSERT(radiusOnField > 0.f);
    ASSERT(std::isnormal(covarianceOnField(0, 0)));
    ASSERT(std::isnormal(covarianceOnField(1, 1)));
    ASSERT(std::isfinite(covarianceOnField(0, 1)));
    ASSERT(std::isfinite(covarianceOnField(1, 0)));
  }
}
