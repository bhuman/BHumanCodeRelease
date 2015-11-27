/**
 * @file KeyStates.h
 *
 * Declaration of struct KeyStates
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

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
    leftFootLeft,
    leftFootRight,
    rightFootLeft,
    rightFootRight,
    chest,
  })

  KeyStates();
  ,
  (std::array<bool, numOfKeys>) pressed,
});

inline KeyStates::KeyStates()
{
  pressed.fill(false);
}