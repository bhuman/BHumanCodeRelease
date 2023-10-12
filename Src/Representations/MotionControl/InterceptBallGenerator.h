/**
 * @file InterceptBallGenerator.h
 *
 * This file declares a representation that ...
 *
 * @author Florian Scholz
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(InterceptBallGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
