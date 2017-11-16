/**
 * @file BallSpecification.h
 *
 * Description of the ball size and rolling friction
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * This representation contains all configurable
 * specifications of the ball.
 */
STREAMABLE(BallSpecification,
{,
  (float)(50)    radius,         //!< The size of the ball (in mm).
  (float)(-0.13) friction,       //!< The friction (negative acceleration) when moving ( in m/(s*s) ).
});
