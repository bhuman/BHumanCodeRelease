/**
* @file KeyStates.h
*
* Declaration of class KeyStates
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
* The class represents the states of the keys.
*/
STREAMABLE(KeyStates,
{
public:
  ENUM(Key,
    rightFootRight,
    rightFootLeft,
    leftFootRight,
    leftFootLeft,
    chest
  ),

  (bool[numOfKeys]) pressed,

  // Initialization
  for(int i = 0; i < numOfKeys; ++i)
    pressed[i] = false;
});
