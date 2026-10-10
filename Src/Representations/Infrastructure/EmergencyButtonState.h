/**
 * @file EmergencyButtonState.h
 *
 * Representation of the emergency button state
 *
 * @author Tom Beuke
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(EmergencyButtonState,
{,
  (bool)(false) buttonPressed, /**< true if button has been pressed */
});
