/**
 * @file InterceptBallInformation.h
 *
 * Declaration of a representation that contains additional information
 * about whether the ball should be avoided if it rolls toward us.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(InterceptBallInformation,
{,
  (bool)(false) intercept,
  (bool)(false) walkOutOfBallPath,
});
