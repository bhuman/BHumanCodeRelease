/**
 * @file GetUpGenerator.h
 *
 * This file declares a representation that can create get up phases.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Streaming/AutoStreamable.h"
#include "Tools/Motion/MotionPhase.h"
#include <memory>

STREAMABLE(GetUpGenerator,
{
  FUNCTION(std::unique_ptr<MotionPhase>(const MotionPhase&)) createPhase,
});
