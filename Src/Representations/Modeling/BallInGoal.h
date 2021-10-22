/**
 * @file BallInGoal.h
 *
 * Information representation about the BallInGoal status.
 *
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(BallInGoal,
{,
  (int)(0) timeSinceLastInGoal,
  (bool)(false) inOwnGoal,
});
