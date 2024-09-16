/**
 * @file JoystickState.h
 *
 * This file defines a representation that contains the state of a joystick,
 * i.e. the state of its buttons and analog sticks.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(JoystickState,
{
  bool pressed(int button) const {return (buttons & 1 << button) != 0;},

  (std::array<float, 8>) axes, /**< The up to 8 axes, each in the range -1 .. 1. */
  (unsigned)(0) buttons, /**< The state of up to 32 buttons (bit set means pressed). */
});
