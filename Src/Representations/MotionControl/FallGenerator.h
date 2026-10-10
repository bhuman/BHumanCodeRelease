/**
 * @file FallGenerator.h
 *
 * This file declares a generator for catching falls.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"
#include <memory>

STREAMABLE_WITH_BASE(FallGenerator, MotionGenerator,
{
  FUNCTION(bool(const MotionRequest&)) shouldCatchFall; /**< Returns whether the fall engine should become active. */
  BASE_HAS_FUNCTION,
});
