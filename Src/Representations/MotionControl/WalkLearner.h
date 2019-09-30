/**
 * @file WalkLearner.h
 *
 * @author Philip Reichenberg
 */

#include "Tools/Function.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/Walk2014Modifier.h"
#pragma once
STREAMABLE(WalkLearner,
{
  FUNCTION(void(float gyroForward, float gyroBackward, float speedTransX)) setBaseWalkParams,

  (float)(0.f) newGyroForwardBalance,
  (float)(0.f) newGyroBackwardBalance,
  (bool)(false) useWalkLearner,
});
