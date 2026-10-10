/**
 * @file PrepareGenerator.h
 *
 * This file declares a representation that brings the robot into a static standing pose.
 * Used for safely moving the robot.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(PrepareGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
