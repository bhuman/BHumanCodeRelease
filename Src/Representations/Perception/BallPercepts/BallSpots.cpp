/**
 * @file BallSpots.cpp
 * Declaration of a struct that represents spots that might be an indication of a ball.
 * @author <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
 * @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
 *
 */

#include "BallSpots.h"
#include "Debugging/DebugDrawings.h"

void BallSpots::draw() const
{
  DEBUG_DRAWING("representation:BallSpots:image", "drawingOnImage") // Draws the ballspots to the image
  {
    for(const Vector2i& ballSpot : ballSpots)
    {
      CROSS("representation:BallSpots:image", ballSpot.x(), ballSpot.y(), 2, 3, Drawings::solidPen, ColorRGBA::orange);
      CROSS("representation:BallSpots:image", ballSpot.x(), ballSpot.y(), 2, 0, Drawings::solidPen, ColorRGBA::black);
    }
  }
}
