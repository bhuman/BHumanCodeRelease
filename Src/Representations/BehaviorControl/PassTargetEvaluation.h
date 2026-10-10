/**
 * @file PassTargetEvaluation.h
 *
 * This file defines a representation that evaluates a target position for passes.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(TargetEvaluation,
{,
  (bool)(false) isValid,
  (bool)(false) isAngleFree,
  (float)(0.f) passRating,
  (float)(0.f) combinedRating,
  (Rangea)(0_deg, 0_deg) precisionRange,
  (Vector2f) ballOnField,
  (Vector2f) targetOnField,
  (Vector2f) bestShiftedTargetOnField,
  (int) teammate,
});

STREAMABLE(PassTargetEvaluation,
{
  FUNCTION(float(const Vector2f& ballOnField, const Vector2f& targetOnField, const int teammate)) getRating;    /**< Estimates the probability that a pass from the base position (e.g. current position) to the given target position would be successful, taking into account the known obstacles. */
  FUNCTION(TargetEvaluation(const Vector2f& ballOnField, const Vector2f& targetOnField, const int teammate)) getBestPassData; /**< Get the best evaluated position around a given target for a pass. */
  FUNCTION(void(const TargetEvaluation& target)) setLastPassTarget,
});
