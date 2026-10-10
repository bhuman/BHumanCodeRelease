/**
 * @file BallKickInformation.h
 *
 * This file defines a representation that gives us information about when the robot touched the ball the last time.
 * It can be used as a pseudo kick timestamp.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

/**
 * @struct BallKickInformation
 */
STREAMABLE(BallKickInformation,
{,
  (unsigned)(0) lastBallContactTimestamp, /**< The last time the robot potentiall touched the ball with its legs. */
});
