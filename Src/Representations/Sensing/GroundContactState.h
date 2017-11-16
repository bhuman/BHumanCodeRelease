/**
 * @file GroundContactState.h
 * Declaration of struct GroundContactState.
 * @author Colin Graf
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct GroundContactState
 * Describes whether we got contact with ground or not.
 */
STREAMABLE(GroundContactState,
{,
  (bool)(false) contact, /**< a foot of the robot touches the ground */
});
