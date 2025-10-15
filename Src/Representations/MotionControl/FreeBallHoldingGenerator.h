/**
 * @file FreeBallHoldingGenerator.h
 *
 * This file declares a representation that can create phases to free a ball held between the feet.
 *
 * @author Harm Thordsen
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(FreeBallHoldingGenerator, MotionGenerator,
{,
});
