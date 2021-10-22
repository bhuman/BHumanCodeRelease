/**
 * @file WalkKickType.h
 *
 * This file declares the walk kick types.
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Tools/Streams/Enum.h"

namespace WalkKicks
{
  ENUM(Type,
  {,
    none,
    forward,
    forwardLong,
    sidewardsOuter,
    turnOut,
    forwardSteal,
    forwardAlternative,
  });
}