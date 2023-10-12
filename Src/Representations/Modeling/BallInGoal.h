/**
 * @file BallInGoal.h
 *
 * Information representation about the BallInGoal status.
 *
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(BallInGoal,
{,
  (unsigned)(0) lastTimeInGoal,
  (bool)(false) inOwnGoal,
});
