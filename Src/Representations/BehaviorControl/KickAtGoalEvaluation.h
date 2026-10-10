/**
 * @file KickAtGoalEvaluation.h
 *
 * This file defines a representation that evaluates kicks at the goal.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"
#include "Representations/Configuration/KickInfo.h"

STREAMABLE(KickAtGoalEvaluation,
{
  STREAMABLE(KickPair,
  {,
    (Angle)(0_deg) kickDirection,                             /**< Relative kick direction. */
    (KickInfo::KickType)(KickInfo::numOfKickTypes) kickType,  /**< Type of kick to be used. */
    (bool)(false) isValid,                                    /**< Is current information valid? */
  });
  FUNCTION(void()) update,                                    /**< Update current best kick. */

  (KickPair) kickInfo,                                        /**< Best planned kick. */
});
