/**
 * @file DampingState.h
 *
 * Contains the information, whether the robot is currently in its damping mode.
 * Only used for Booster robots.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(DampingState,
{,
  (bool)(false) isDamping,
});
