/**
 * @file FreezeGenerator.h
 *
 * This file declares a generator for freezing the joint request.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionPhase.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(FreezeGenerator,
{
  FUNCTION(bool(const MotionPhase& currentPhase)) shouldHandleBodyDisconnect; /**< Returns whether the fall engine should become active. */
  FUNCTION(std::unique_ptr<MotionPhase>()) createPhase, /**< Creates a fall phase. */
});
