/**
 * @file WalkAtRelativeSpeedGenerator.h
 *
 * This file declares a representation ...
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(WalkAtRelativeSpeedGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
