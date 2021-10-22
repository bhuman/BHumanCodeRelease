/**
 * @file FallGenerator.h
 *
 * This file declares a generator for catching falls.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Streams/AutoStreamable.h"
#include <memory>

STREAMABLE(FallGenerator,
{
  FUNCTION(bool(const MotionRequest&)) shouldCatchFall; /**< Returns whether the fall engine should become active. */
  FUNCTION(std::unique_ptr<MotionPhase>()) createPhase, /**< Creates a fall phase. */
});
