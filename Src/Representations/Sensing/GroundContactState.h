/**
* @file GroundContactState.h
* Declaration of class GroundContactState.
* @author Colin Graf
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
* @class GroundContactState
* Describes whether we got contact with ground or not.
*/
STREAMABLE(GroundContactState,
{,
  (bool)(true) contact, /**< a foot of the robot touches the ground */
});
