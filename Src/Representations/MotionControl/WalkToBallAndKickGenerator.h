/**
 * @file WalkToBallAndKickGenerator.h
 *
 * This file declares a representation that can create phases to walk to the ball and kick.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(WalkToBallAndKickGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
