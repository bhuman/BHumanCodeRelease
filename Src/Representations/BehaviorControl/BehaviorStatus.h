/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus,
{
  ENUM(Activity,
  {,
    unknown,
    blocking,
    duel,
    dribble,
    dribbleDuel,
    searchForBall,
    searchForPercept,
    goToBall,
    takingPosition,
    kick,
    kickoffkick,
    guardGoal,
    catchBall,
    waitingForPass,
    standAndWait,
    passing,
    gettingUp,
    turn,
    zeroValidityTurn,
    checkMirroredBall,
    walkNextToKeeper,
    kickoff,
    waving,
    noWifi,
    noWifiLocation,
    noWifiData,
  });

  ENUM(TeamColor,
  {,
    red,
    blue,
  }),

  ((Role) RoleType)(undefined) role,
  (Activity)(unknown) activity, /**< What is the robot doing in general? */
  (int)(-1) passTarget,
});
