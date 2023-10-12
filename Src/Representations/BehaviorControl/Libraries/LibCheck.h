/**
 * @file LibCheck.h
 *
 * This file defines a representation that checks some behavior control properties
 *
 * @author Daniel Krause
 */

#pragma once

#include "Streaming/Function.h"
#include "RobotParts/Arms.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(LibCheck,
{
  ENUM(CheckedOutput,
  {,
    motionRequest,
    headMotionRequest,
    calibrationFinished,
    passTarget,
  });

  /** Increments one counter */
  FUNCTION(void(LibCheck::CheckedOutput outputToCheck)) inc;

  /** Decrements one counter. Only use if an output should really be overwritten. */
  FUNCTION(void(LibCheck::CheckedOutput outputToCheck)) dec;

  /** Performs checks for the individual behavior */
  FUNCTION(void()) performCheck,
});
