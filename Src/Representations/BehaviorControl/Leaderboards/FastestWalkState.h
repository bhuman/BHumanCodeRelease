/**
 * @file FastestWalkState.h
 *
 * This file defines a representation that contains the state of the fastest walk leaderboard challenge.
 *
 * @author Moritz Oppermann
 */

#pragma once
#include "Streaming/Streamable.h"

STREAMABLE(FastestWalkState,
{
  ENUM(WalkState,
  {,
    forward,
    slalom,
  }),

  (FastestWalkState::WalkState)(FastestWalkState::forward) state,
});
