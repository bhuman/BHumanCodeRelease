/**
 * @file WalkAtAbsoluteSpeedGenerator.h
 *
 * This file declares a representation that can create phases to walk at a relative speed.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(WalkAtAbsoluteSpeedGenerator, MotionGenerator,
{
  FUNCTION(std::unique_ptr<MotionPhase>(const Pose2f& walkSpeed, const MotionPhase& lastPhase)) createPhaseToTarget,
});
