/**
 * @file KickLengthPair.h
 *
 * TODO
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Eigen.h"

STREAMABLE(KickLengthPair,
{
  STREAMABLE(Pair,
  {,
    (float) rangeIncrease, /**< The % range relative to the max range. */
    (float) powerIncrease, /**< The power value. */
  }),

  (std::vector<Pair>) kickLengthDynamicOldPair,
});
