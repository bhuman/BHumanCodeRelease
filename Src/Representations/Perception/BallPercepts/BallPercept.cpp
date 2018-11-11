/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @autor Jesse
 */

#include "BallPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "robot");
  TRANSLATE3D("representation:BallPercept", 0, 0, -230);
  if(status == seen)
  {
    CIRCLE("representation:BallPercept:image", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
           Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 128, 64, 100));
    CIRCLE("representation:BallPercept:field", positionOnField.x(), positionOnField.y(), radiusOnField, 0, // pen width
           Drawings::solidPen, ColorRGBA::orange, Drawings::noBrush, ColorRGBA::orange);
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
    ASSERT(radiusInImage > 0.f);
    ASSERT(radiusOnField > 0.f);
  }
}
