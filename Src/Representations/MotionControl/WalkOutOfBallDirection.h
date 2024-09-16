/**
 * @file WalkOutOfBallDirection.h
 *
 * This file declares a representation that ...
 *
 * @author Sina Schreiber
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(WalkOutOfBallDirection,
{
  FUNCTION(std::unique_ptr<MotionPhase>(const MotionRequest&, const MotionPhase&, const MotionPhase&)) createPhase, /**< Creates a phase for the given motion request. */
});
