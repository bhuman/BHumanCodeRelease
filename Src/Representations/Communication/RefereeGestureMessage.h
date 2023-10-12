/**
 * @file RefereeGestureMessage.h
 *
 * This file defines a representation that <#...#>
 *
 * @author Ayleen Lührsen
 */

#pragma once

#include "Representations/Perception/RefereePercept/RefereePercept.h"

STREAMABLE(RefereeGestureMessage,
{,
  (bool)(false) sendMessage,
  (unsigned int)(0) messageDelay,
  (RefereePercept::Gesture)(RefereePercept::none) recognizedGesture,
  (unsigned int)(0) timeStamp,
});
