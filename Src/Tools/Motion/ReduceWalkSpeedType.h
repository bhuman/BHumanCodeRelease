/**
 * @file ReduceWalkSpeed.h
 * This file contains information about how the walk speed be reduced
 *
 * @author Philip Reichenberg
 */

#pragma once
#include "Streaming/Enum.h"

namespace ReduceWalkSpeedType
{
  ENUM(ReduceWalkSpeedType,
  {,
    noChange, /**< No reduction is applied. */
    normal, /**< Reduce down to a normal, non competition like, walking speed. */
    slow, /**< Reduce down to a slow walk speed. */
    timeBased, /**< Reduce the speed based on the time left in current state. */
    distanceBased, /**< Reduce the speed based on the distance to the target. */
  });
}
