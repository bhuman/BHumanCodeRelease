/**
 * @file PlayerRole.h
 *
 * This file declares a representation of a player's role.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include <string>

STREAMABLE(PlayerRole,
{
  ENUM(RoleType,
  {,
    none,
    goalkeeper,
    ballPlayer,

    // legacy roles from 2019
    firstSupporterRole,
    supporter0 = firstSupporterRole,
    supporter1,
    supporter2,
    supporter3,
    supporter4,
  });

  /**
   * Compatibility function for 2019 behavior.
   * @return Whether the robot is goalkeeper.
   */
  bool isGoalkeeper() const
  {
    return role == goalkeeper;
  }

  /**
   * Compatibility function for 2019 behavior.
   * @return Whether the robot plays the ball.
   */
  bool playsTheBall() const
  {
    return role == ballPlayer;
  }

  /**
   * Compatibility function for 2019 behavior.
   * @return The robot's supporter index.
   */
  int supporterIndex() const
  {
    return role < firstSupporterRole ? -1 : (role - firstSupporterRole);
  },

  (RoleType)(none) role, /**< The role type. */
  (int)(0) numOfActiveSupporters, /**< The number of not penalized supporters (i.e. robots that have a supporterIndex >= 0). */
});
