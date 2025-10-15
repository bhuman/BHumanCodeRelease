/**
 * @file BestKickState.h
 *
 * This file defines a representation that contains the state of the best kick leaderboard challenge.
 *
 * @author Moritz Oppermann
 */

#pragma once
#include "Streaming/Streamable.h"

STREAMABLE(BestKickState,
{
  ENUM(Target,
  {,
    a,
    b,
    c,
  });,

  (BestKickState::Target) target,
});
