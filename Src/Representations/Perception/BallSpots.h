/**
 * @file BallSpots.h
 * Declaration of a struct that represents a spot that might be an indication of a ball.
 * @author <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
 * @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
 *
 */

#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "BallSpot.h"

/**
 * @struct BallSpots
 * A struct that represents a spot that might be an indication of a ball.
 */
STREAMABLE(BallSpots,
{
  BallSpots()
  {
    ballSpots.reserve(50);
  }

  void addBallSpot(int x, int y)
  {
    ballSpots.emplace_back(x, y);
  }

  /** The method draws all ball spots. */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:BallSpots:Image", "drawingOnImage"); // Draws the ballspots to the image
    COMPLEX_DRAWING("representation:BallSpots:Image")
    {
      for(std::vector<BallSpot>::const_iterator i = ballSpots.begin(); i != ballSpots.end(); ++i)
      {
        CROSS("representation:BallSpots:Image", i->position.x(), i->position.y(), 2, 3, Drawings::solidPen, ColorRGBA::orange);
        CROSS("representation:BallSpots:Image", i->position.x(), i->position.y(), 2, 0, Drawings::solidPen, ColorRGBA::black);
      }
    }
  },

  (std::vector<BallSpot>) ballSpots,
});
