/**
 * @file Representations/MotionControl/ReplayWalkRequestGenerator.h
 * @author Philip Reichenberg
 */

#pragma once
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(ReplayWalkRequestGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION;

  FUNCTION(void(const MotionPhase& lastPhase)) savePhase,
});
