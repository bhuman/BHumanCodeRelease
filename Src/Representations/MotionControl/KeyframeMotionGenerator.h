/**
 * @file KeyframeMotionGenerator.h
 *
 * This file declares a representation that can create keyframe motion phases.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"
#include "Tools/Motion/MotionPhase.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"
#include <memory>

STREAMABLE(KeyframeMotionGenerator,
{
  FUNCTION(std::unique_ptr<MotionPhase>(const KeyframeMotionRequest&, const MotionPhase&)) createPhase;
  FUNCTION(void(const float ratio, const KeyframeMotionRequest::KeyframeMotionID motion)) setPhaseInformation;
  FUNCTION(bool(const MotionPhase&)) wasLastGetUp,

  (float)(0.f) ratio,
  (KeyframeMotionRequest::KeyframeMotionID)(KeyframeMotionRequest::decideAutomatic) motion,
});
