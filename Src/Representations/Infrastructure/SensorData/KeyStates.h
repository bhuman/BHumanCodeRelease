/**
 * @file KeyStates.h
 *
 * Declaration of struct KeyStates
 */

#pragma once

#include "Tools/Streams/EnumIndexedArray.h"

/**
 * The struct represents the states of the keys.
 */
STREAMABLE(KeyStates,
{
  ENUM(Key,
  {,
    // touch sensors:
    headFront,
    headMiddle,
    headRear,
    lHandBack,
    lHandLeft,
    lHandRight,
    rHandBack,
    rHandLeft,
    rHandRight,

    // bumpers:
    lFootLeft,
    lFootRight,
    rFootLeft,
    rFootRight,
    chest,
  });

  KeyStates(),

  (ENUM_INDEXED_ARRAY(bool, Key)) pressed,
});

inline KeyStates::KeyStates()
{
  pressed.fill(false);
}
