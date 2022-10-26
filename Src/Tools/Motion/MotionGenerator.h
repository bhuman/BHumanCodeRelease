/**
 * @file MotionGenerator.h
 *
 * This file declares a base class for representations that generate motion phases.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"
#include "Tools/Motion/MotionPhase.h"
#include <memory>

STREAMABLE(MotionGenerator,
{
  FUNCTION(std::unique_ptr<MotionPhase>(const MotionRequest&, const MotionPhase&)) createPhase, /**< Creates a phase for the given motion request. */
});
