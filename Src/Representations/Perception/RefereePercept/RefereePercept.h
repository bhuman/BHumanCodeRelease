#pragma once
/**
 * @file RefereePercept.h
 *
 * Very simple representation of the referee gesture.
 *
 * @author <a href="mailto:aylu@uni-bremen.de">Ayleen Lührsen</a>
 */

#include "Streaming/Enum.h"

STREAMABLE(RefereePercept,
{
  ENUM(Gesture,
  {,
    none,
    kickInBlue,
    kickInRed,
    goalKickBlue,
    goalKickRed,
    cornerKickBlue,
    cornerKickRed,
    goalBlue,
    goalRed,
    pushingFreeKickBlue,
    pushingFreeKickRed,
    fullTime,
    substitution,
    initialToReady,
  }),

  (Gesture)(none) gesture,
});
