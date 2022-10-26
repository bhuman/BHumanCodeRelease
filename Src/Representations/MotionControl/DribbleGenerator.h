/**
 * @file DribbleGenerator.h
 *
 * This file declares a representation that can create phases to dribble the ball.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(DribbleGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
