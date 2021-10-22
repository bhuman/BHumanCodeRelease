/**
 * @file GetUpGenerator.h
 *
 * This file declares a representation that can create get up phases.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(GetUpGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
