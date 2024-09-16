/**
 * @file ExpectedGoals.h
 *
 * This file defines a representation that estimates the probability of scoring a goal when shooting from a hypothetical ball position.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(ExpectedGoals,
{
  FUNCTION(float(const Vector2f& pointOnField)) xG; /**< Estimates the probability of scoring a goal when shooting from a given position, not taking into account the known obstacles. */
  FUNCTION(float(const Vector2f& pointOnField)) xGA; /**< Estimates the probability of an opponent missing the goal when shooting from a given position, not taking into account the known obstacles. */
  FUNCTION(float(const Vector2f& pointOnField, const bool isPositioning)) getRating; /**< Estimates the probability that a given position has a wide enough opening angle on the opponent's goal to score a goal, taking into account the known obstacles. */
  FUNCTION(float(const Vector2f& pointOnField)) getOpponentRating, /**< Estimates the probability that a given position does not have a wide enough opening angle on the own goal for an opponent to score a goal against, not taking into account the known obstacles. */
});
