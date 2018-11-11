/**
 * @file BallSpots.h
 * Declaration of a struct that represents spots that might be an indication of a ball.
 * @author <a href="mailto:jworch@informatik.uni-bremen.de">Jan-Hendrik Worch</a>
 * @author <a href="mailto:ingsie@informatik.uni-bremen.de">Ingo Sieverdingbeck</a>
 *
 */

#pragma once

#include "Tools/Debugging/DebugDrawings.h"

/**
 * @struct BallSpots
 * A struct that represents spots that might be an indication of a ball.
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
  void draw() const,

  (std::vector<Vector2i>) ballSpots,
  (bool) firstSpotIsPredicted, /**< true if the first ball spot is derived from the ball model */
});
