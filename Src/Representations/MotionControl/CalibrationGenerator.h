/**
 * @file CalibrationGenerator.h
 *
 * This file declares a representation that can create phases to calibrate the robot.
 * The robot is meant to start from a standing position.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(CalibrationGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
