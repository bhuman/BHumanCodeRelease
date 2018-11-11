/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Role.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus,
{
  ENUM(Activity,
  {,
    unknown,
    searchForBall,
    searchForBallAtRecentPosition,
    goToBall,
    takingPosition,
    kick,
    guardGoal,
    catchBall,
    standAndWait,
    gettingUp,
    turn,
    kickoff,

    demo,
  }),

  (Role::RoleType)(Role::undefined) role,
  (Activity)(unknown) activity, /**< What is the robot doing in general? */
  (int)(-1) passTarget,
  (Vector2f)(Vector2f::Zero()) walkingTo,
  (Vector2f)(Vector2f::Zero()) shootingTo,
});
