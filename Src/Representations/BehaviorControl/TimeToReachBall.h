/**
 * @file Representations/BehaviorControl/TimeToReachBall.h
 *
 * Representation of the estimated time to reach the ball
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <limits>

/**
 * @struct TimeToReachBall
 * Representation of the Time to reach the ball
 */
STREAMABLE(TimeToReachBall,
{,
  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBall,         /**< The estimated time when reach the ball */
  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBallStriker,
});
