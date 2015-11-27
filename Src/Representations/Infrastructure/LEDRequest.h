/**
 * @file Representations/Infrastructure/LEDRequest.h
 *
 * This file contains the LEDRequest struct.
 *
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

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
    headLedRearLeft0,
    headLedRearLeft1,
    headLedRearLeft2,
    headLedRearRight0,
    headLedRearRight1,
    headLedRearRight2,
    headLedMiddleRight0,
    headLedFrontRight0,
    headLedFrontRight1,
    headLedFrontLeft0,
    headLedFrontLeft1,
    headLedMiddleLeft0,
    footLeftRed,
    footLeftGreen,
    footLeftBlue,
    footRightRed,
    footRightGreen,
    footRightBlue,
  });

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
    for(int i = 0; i < numOfLEDs; ++i)
      ledStates[i] = off;
  }

  bool operator==(const LEDRequest& other) const
  {
    for(int i = 0; i < numOfLEDs; i++)
      if(ledStates[i] != other.ledStates[i])
        return false;
    return true;
  }

  bool operator!=(const LEDRequest& other) const
  {
    return !(*this == other);
  }
  ,
  (LEDState[numOfLEDs]) ledStates, /**< The intended states of the LEDs (use type State). */
});
