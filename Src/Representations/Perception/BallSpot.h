/**
* @file BallSpot.h
* Declaration of a class that represents a spot that might be an indication of a ball.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class BallSpot
* A class that represents a spot that might be an indication of a ball.
*/
STREAMABLE(BallSpot,
{
public:
  BallSpot(int x, int y)
  {
    position.x = x;
    position.y = y;
  },

  (Vector2<int>) position,
});
