/**
 * @file BallSpecification.h
 *
 * Description of the ball size and rolling behavior
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * This representation contains all configurable
 * specifications of the ball.
 */
STREAMABLE(BallSpecification,
{
public:
  /** Computes a maximum ball speed that is above any actual possible speed occurring in a real
   *  SPL game. The computation is based on the configured kickDistanceUpperBound. An alternative
   *  would have been to configure the speed and to perform no computation. However, the velocity
   *  is more difficult to observe and configure than any distance. Thus, this way is more foolproof.
   * @return What is described above
   */
  float ballSpeedUpperBound() const
  {
    return BallPhysics::velocityForDistance(kickDistanceUpperBound, friction);
  },

  (float)(50.f)    radius,                 //!< The size of the ball (in mm).
  (float)(-0.13f)  friction,               //!< The friction (negative acceleration) when moving ( in m/(s*s) ).
  (float)(20000.f) kickDistanceUpperBound, //!< A distance (in mm) that is larger (but within the same order of magnitude) than the longest possible kick of any NAO robot.
});
