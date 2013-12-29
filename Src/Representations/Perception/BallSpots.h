/**
* @file BallSpots.h
* Declaration of a class that represents a spot that might be an indication of a ball.
* @author <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
* @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
*
*/

#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Vector2.h"
#include "BallSpot.h"

/**
* @class BallSpots
* A class that represents a spot that might be an indication of a ball.
*/
STREAMABLE(BallSpots,
{
public:
  void addBallSpot(int x, int y)
  {
    BallSpot bs;
    bs.position.x = x;
    bs.position.y = y;
    ballSpots.push_back(bs);
  }

  /** The method draws all ball spots. */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:BallSpots:Image", "drawingOnImage"); // Draws the ballspots to the image
    COMPLEX_DRAWING("representation:BallSpots:Image",
    {
      for(std::vector<BallSpot>::const_iterator i = ballSpots.begin(); i != ballSpots.end(); ++i)
      {
        CROSS("representation:BallSpots:Image", i->position.x, i->position.y, 2, 3, Drawings::ps_solid, ColorClasses::orange);
        CROSS("representation:BallSpots:Image", i->position.x, i->position.y, 2, 0, Drawings::ps_solid, ColorClasses::black);
      }
    });
  },

  (std::vector<BallSpot>) ballSpots,

  // Initialization
  ballSpots.reserve(50);
});
