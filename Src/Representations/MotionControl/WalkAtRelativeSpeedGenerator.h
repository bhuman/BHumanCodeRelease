/**
 * @file WalkAtRelativeSpeedGenerator.h
 *
 * This file declares a representation ...
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(WalkAtRelativeSpeedGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
