/**
 * @file KeyframeMotionGenerator.h
 *
 * This file declares a representation that can create keyframe motion phases.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/MotionControl/KeyframeMotionRequest.h"

STREAMABLE(EditKeyframe,
{,
  (JointAngles)(JointAngles()) angles,
  (KeyframeMotionRequest::KeyframeMotionID)(KeyframeMotionRequest::decideAutomatic) motion,
  (int)(-1) line,
});

STREAMABLE_WITH_BASE(KeyframeMotionGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION;
  FUNCTION(void(const int line, const float ratio, const KeyframeMotionRequest::KeyframeMotionID motion)) setPhaseInformation,

  (int)(0) line,
  (float)(0.f) ratio,
  (KeyframeMotionRequest::KeyframeMotionID)(KeyframeMotionRequest::decideAutomatic) motion,
  (EditKeyframe) editKeyframe,
});
