/**
 * @file Representations/Infrastructure/LEDRequest.h
 *
 * This file contains the LEDRequest struct.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/EnumIndexedArray.h"

/**
 * This describes a LEDRequest
 */
STREAMABLE(LEDRequest,
{
  /** ids for all LEDs */
  ENUM(LED,
  {,
    red,
    green,
    blue,
  });

  ENUM(State,
  {,
    off,
    on,
    blinking,
    fastBlinking,
    half,
    halfBlinking,
    halfFastBlinking,
  });

  LEDRequest()
  {
    FOREACH_ENUM(LED, i)
      ledStates[i] = off;
  }

  bool operator==(const LEDRequest& other) const
  {
    FOREACH_ENUM(LED, i)
      if(ledStates[i] != other.ledStates[i])
        return false;
    return true;
  }

  bool operator!=(const LEDRequest& other) const
  {
    return !(*this == other);
  },

  (ENUM_INDEXED_ARRAY(LEDRequest::State, LED)) ledStates, /**< The intended states of the LEDs. */
});
