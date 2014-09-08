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
    CIRCLE("representation:BallPercept:Image",
           positionInImage.x,
           positionInImage.y,
           radiusInImage,
           1, // pen width
           Drawings::ps_solid,
           ColorRGBA::black,
           Drawings::bs_solid,
           ColorRGBA(255, 128, 64, 100));
    CIRCLE("representation:BallPercept:Field",
           relativePositionOnField.x,
           relativePositionOnField.y,
           35,
           0, // pen width
           Drawings::ps_solid,
           ColorRGBA::orange,
           Drawings::bs_null,
           ColorRGBA::orange);
    // Sorry, no access to field dimensions here, so ball radius is hard coded
    SPHERE3D("representation:BallPercept", relativePositionOnField.x, relativePositionOnField.y, 35, 35, ColorRGBA::orange);
  }
  else if(status != notSeen)
  {
    CROSS("representation:BallPercept:Image",
          positionInImage.x, positionInImage.y,
          1, 1, Drawings::ps_solid, ColorRGBA::black);
    DRAWTEXT("representation:BallPercept:Image", positionInImage.x + 3, positionInImage.y + 2, 10, ColorRGBA::black,
             BallPercept::getName(status));
  }
}
