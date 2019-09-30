/**
 * @file ConfirmedBallSpot.h
 *
 * This file defines a representation that stores a single ball spot that
 * was identified as being in the vicinity of the actual ball.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "BallPercept.h"

/** Potential balls that need a final validation. **/
STREAMABLE(DemoConfirmedBallSpots,
{,
 (std::vector<Vector2i>) positionsInImage,
});

STREAMABLE(ConfirmedBallSpot,
{,
  (Vector2i) positionInImage,
  (BallPercept::Status)(BallPercept::notSeen) status,
});
