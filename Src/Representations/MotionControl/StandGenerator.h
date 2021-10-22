/**
 * @file StandGenerator.h
 *
 * This file declares a representation that can create stand phases.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(StandGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
