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
  FUNCTION(float(const Vector2f& baseOnField, const Vector2f& targetOnField, const bool isPositioning)) getRating, /**< Estimates the probability that a pass from the base position (e.g. current position) to the given target position would be successful, taking into account the known obstacles. */
});
