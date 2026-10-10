/**
 * @file BallSpecification.h
 *
 * Description of the properties of a ball
 *
 * @author Tim Laue
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Streaming/AutoStreamable.h"

/**
 * This representation contains all configurable
 * specifications of the ball.
 */
STREAMABLE(BallSpecification,
{
  /** Verifies that all elements contain valid values */
  void verify() const
  {
    ASSERT(radius > 0.f);
    ASSERT(staticFriction < 0.0f);
    ASSERT(linearFriction <= 0.f);
    ASSERT(ballSpeedUpperBound > 0.f);
    ASSERT(mass > 0.f);
  },

  (float)(50.f)    radius,                //!< The size of the ball (in mm).
  (float)(-0.13f)  staticFriction,        //!< Constant friction (negative acceleration) when moving ( in m/(s*s) ).
  (float)(0.f)     linearFriction,        //!< Linear friction (negative acceleration depending on current velocity) when moving ( in 1/s ).
  (float)(35000.f) ballSpeedUpperBound,   //!< A speed (in mm/s). Default value is above Booster T2 in 2026.
  (float)(0.3f)    mass,                  //!< The mass of the ball (in kg)
});
