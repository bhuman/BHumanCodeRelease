/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#include "BallPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "robot");
  TRANSLATE3D("representation:BallPercept", 0, 0, -230);
  if(status == seen)
  {
    CIRCLE("representation:BallPercept:Image", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
           Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 128, 64, 100));
    CIRCLE("representation:BallPercept:Field", relativePositionOnField.x(), relativePositionOnField.y(), radiusOnField, 0, // pen width
           Drawings::solidPen, ColorRGBA::orange, Drawings::noBrush, ColorRGBA::orange);
    SPHERE3D("representation:BallPercept", relativePositionOnField.x(), relativePositionOnField.y(), radiusOnField, radiusOnField, ColorRGBA::orange);
  }
  else if(status != notSeen)
  {
    CROSS("representation:BallPercept:Image", positionInImage.x(), positionInImage.y(), 1, 1, Drawings::solidPen, ColorRGBA::black);
    DRAWTEXT("representation:BallPercept:Image", positionInImage.x() + 3, positionInImage.y() + 2, 10, ColorRGBA::black, BallPercept::getName(status));
  }
}
