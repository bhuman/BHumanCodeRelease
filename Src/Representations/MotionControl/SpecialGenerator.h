/**
 * @file SpecialGenerator.h
 *
 * This file declares a representation that can create phases for special motions.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(SpecialGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
