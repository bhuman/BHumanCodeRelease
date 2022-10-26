#pragma once
/**
 * @file RefereePercept.h
 *
 * Very simple representation of the referee position
 *
 * @author <a href="mailto:aylu@uni-bremen.de">Ayleen Lührsen</a>
 */

#include "Streaming/Enum.h"
#include "Math/Eigen.h"

STREAMABLE(RefereePercept,
{
  // The gestures that the referee indicates, where red is the left team and blue is the right team
  ENUM(Gesture,
  {,
    kickInRed,
    kickInBlue,
    goalKickRed,
    goalKickBlue,
    cornerKickRed,
    cornerKickBlue,
    goalRed,
    goalBlue,
    pushingFreeKickRed,
    pushingFreeKickBlue,
    fullTime,
    none,
  }),

  /* default gesture is none.A new one is assigned in RefereeGestureDetection#update.
   If the robot is unpenalized and a whistle was heard, as soon as another valid gesture was assigned to the percept, the robot will imitate the pose, triggered by the Option HandleRefereePose  */
  (Gesture)(none) gesture,
});

