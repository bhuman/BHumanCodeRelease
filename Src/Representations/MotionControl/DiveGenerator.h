/**
 * @file DiveGenerator.h
 *
 * This file declares a representation that can create phases to dive.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(DiveGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
