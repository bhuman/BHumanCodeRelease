/**
 * @file BallHoldingState.h
 * @author Harm Thordsen
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(BallHoldingState,
{,
  (bool)(false) ballHolding,
});
