/**
* @file Representations/BehaviorControl/BehaviorLEDRequest.h
* This file contains the BehaviorLEDRequest class.
* @author jeff
*/

#pragma once

#include "Representations/Infrastructure/LEDRequest.h"

STREAMABLE(BehaviorLEDRequest,
{
public:
  ENUM(BehaviorLED,
    leftEye,
    rightEye,
    leftEar,
    rightEar
  );

  ENUM(EyeColor,
    defaultColor,
    red,
    green,
    blue,
    white,
    magenta,
    yellow,
    cyan
  );

  bool operator==(const BehaviorLEDRequest& other) const
  {
    for(int i = 0; i < numOfBehaviorLEDs; i++)
      if(modifiers[i] != other.modifiers[i])
        return false;
    return true;
  }

  bool operator!=(const BehaviorLEDRequest& other) const
  {
    return !(*this == other);
  },

  (LEDRequest, LEDState[numOfBehaviorLEDs]) modifiers,
  (EyeColor)(defaultColor) leftEyeColor,
  (EyeColor)(defaultColor) rightEyeColor,

  // Initialization
  for(int i = 0; i < numOfBehaviorLEDs; ++i)
    modifiers[i] = LEDRequest::on;
});
