/**
 * @file Representations/Infrastructure/LEDRequest.h
 *
 * This file contains the LEDRequest struct.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/EnumIndexedArray.h"

/**
 * This describes a LEDRequest
 */
STREAMABLE(LEDRequest,
{
  /** ids for all LEDs */
  ENUM(LED,
  {,
    faceLeftRed0Deg,
    faceLeftRed45Deg,
    faceLeftRed90Deg,
    faceLeftRed135Deg,
    faceLeftRed180Deg,
    faceLeftRed225Deg,
    faceLeftRed270Deg,
    faceLeftRed315Deg,
    faceLeftGreen0Deg,
    faceLeftGreen45Deg,
    faceLeftGreen90Deg,
    faceLeftGreen135Deg,
    faceLeftGreen180Deg,
    faceLeftGreen225Deg,
    faceLeftGreen270Deg,
    faceLeftGreen315Deg,
    faceLeftBlue0Deg,
    faceLeftBlue45Deg,
    faceLeftBlue90Deg,
    faceLeftBlue135Deg,
    faceLeftBlue180Deg,
    faceLeftBlue225Deg,
    faceLeftBlue270Deg,
    faceLeftBlue315Deg,
    faceRightRed0Deg,
    faceRightRed45Deg,
    faceRightRed90Deg,
    faceRightRed135Deg,
    faceRightRed180Deg,
    faceRightRed225Deg,
    faceRightRed270Deg,
    faceRightRed315Deg,
    faceRightGreen0Deg,
    faceRightGreen45Deg,
    faceRightGreen90Deg,
    faceRightGreen135Deg,
    faceRightGreen180Deg,
    faceRightGreen225Deg,
    faceRightGreen270Deg,
    faceRightGreen315Deg,
    faceRightBlue0Deg,
    faceRightBlue45Deg,
    faceRightBlue90Deg,
    faceRightBlue135Deg,
    faceRightBlue180Deg,
    faceRightBlue225Deg,
    faceRightBlue270Deg,
    faceRightBlue315Deg,
    earsLeft0Deg,
    earsLeft36Deg,
    earsLeft72Deg,
    earsLeft108Deg,
    earsLeft144Deg,
    earsLeft180Deg,
    earsLeft216Deg,
    earsLeft252Deg,
    earsLeft288Deg,
    earsLeft324Deg,
    earsRight0Deg,
    earsRight36Deg,
    earsRight72Deg,
    earsRight108Deg,
    earsRight144Deg,
    earsRight180Deg,
    earsRight216Deg,
    earsRight252Deg,
    earsRight288Deg,
    earsRight324Deg,
    chestRed,
    chestGreen,
    chestBlue,
    firstHeadLED,
    headRearLeft0 = firstHeadLED,
    headRearLeft1,
    headRearLeft2,
    headRearRight0,
    headRearRight1,
    headRearRight2,
    headMiddleRight0,
    headFrontRight0,
    headFrontRight1,
    headFrontLeft0,
    headFrontLeft1,
    headMiddleLeft0,
    lastHeadLED = headMiddleLeft0,
    footLeftRed,
    footLeftGreen,
    footLeftBlue,
    footRightRed,
    footRightGreen,
    footRightBlue,
  });

  static constexpr size_t numOfHeadLEDs = LED::headMiddleLeft0 - LED::headRearLeft0 + 1;

  ENUM(LEDState,
  {,
    off,
    on,
    blinking,
    fastBlinking,
    half,
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

  (ENUM_INDEXED_ARRAY(LEDRequest::LEDState, LED)) ledStates, /**< The intended states of the LEDs (use type State). */
});
