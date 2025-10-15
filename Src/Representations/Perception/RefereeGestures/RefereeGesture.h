/**
 * @file RefereeGesture.h
 *
 * Very simple representation of the referee gesture.
 *
 * @author <a href="mailto:aylu@uni-bremen.de">Ayleen Lührsen</a>
 */

#pragma once

#include "Streaming/Enum.h"

STREAMABLE(RefereeGesture,
{
  ENUM(Gesture,
  {,
    none,
    kickInLeft,
    kickInRight,
    ready,
  }),

  (Gesture)(none) gesture,
  (float)(0.0f) netout1,
  (float)(0.0f) netout2,
  (float)(0.0f) netout3,
  (float)(0.0f) normDist,
});
