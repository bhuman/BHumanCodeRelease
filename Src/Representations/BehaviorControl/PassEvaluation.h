/**
 * @file PassEvaluation.h
 *
 * This file defines a representation that evaluates a target position by estimating how likely it is that a pass of the ball would be successful.
 *
 * @author Jo Lienhoop
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(PassEvaluation,
{
  FUNCTION(float(const Vector2f& pointOnField)) getRating, /**< Estimates the probability that a pass from the current ball position to the given target position would be successful, taking into account the known obstacles. */
});
