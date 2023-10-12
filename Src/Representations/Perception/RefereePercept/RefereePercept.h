#pragma once
/**
 * @file RefereePercept.h
 *
 * Very simple representation of the referee gesture.
 *
 * @author <a href="mailto:aylu@uni-bremen.de">Ayleen Lührsen</a>
 */

#include "Streaming/Enum.h"
#include <VisualRefereeChallenge.h>

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
    substitutionBlue = substitution,
    substitutionRed,
  });

  static_assert(kickInBlue == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_KICK_IN_BLUE_TEAM);
  static_assert(kickInRed == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_KICK_IN_RED_TEAM);
  static_assert(goalKickBlue == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_KICK_BLUE_TEAM);
  static_assert(goalKickRed == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_KICK_RED_TEAM);
  static_assert(cornerKickBlue == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_CORNER_KICK_BLUE_TEAM);
  static_assert(cornerKickRed == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_CORNER_KICK_RED_TEAM);
  static_assert(goalBlue == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_BLUE_TEAM);
  static_assert(goalRed == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_GOAL_RED_TEAM);
  static_assert(pushingFreeKickBlue == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_PUSHING_FREE_KICK_BLUE_TEAM);
  static_assert(pushingFreeKickRed == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_PUSHING_FREE_KICK_RED_TEAM);
  static_assert(fullTime == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_FULL_TIME);
  static_assert(substitutionBlue == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_SUBSTITUTION_BLUE_TEAM);
  static_assert(substitutionRed == GAMECONTROLLER_RETURN_STRUCT_VRC_GESTURE_SUBSTITUTION_RED_TEAM),

  (Gesture)(none) gesture,
});
