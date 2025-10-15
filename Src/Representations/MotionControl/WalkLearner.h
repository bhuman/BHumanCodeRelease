/**
 * @file WalkLearner.h
 *
 * @author Philip Reichenberg
 */

#include "Streaming/Function.h"
#include "Representations/MotionControl/WalkGenerator.h"
#pragma once
STREAMABLE(WalkLearner,
{
  FUNCTION(void(const Pose2f& lastStep, const float lastDuration, const float refStepDuration)) updateStepDuration,

  (float)(125.f) stepHeightDurationOffset, /**< Additional offset for the height interpolation, to increase the interpolation time. */
});
