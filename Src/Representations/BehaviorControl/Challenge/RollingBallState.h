/**
 * @file RollingBallState.h
 *
 * This file defines a representation that contains the state of the rolling ball challenge
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Streaming/Streamable.h"

STREAMABLE(RollingBallState,
{,
  (bool)(false) isActive, /**< Is the challenge active? */
  (bool)(false) isRampLeftSide, /**< Is the ramp on the left side? */
  (float)(750.f) approxRampDistance, /**< Approximate distance to the ramp. */
  (float)(3000.f) maxRampDistance, /**< Max expected ramp distance. */
});
