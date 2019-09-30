/**
 * @file Representations/MotionControl/MotionRequest.h
 * This file declares a struct that represents the motions that can be requested from the robot.
 * @author Thomas Röfer
 */

#pragma once

#include "SpecialActionRequest.h"
#include "WalkRequest.h"
#include "KickRequest.h"

/**
 * @struct MotionRequest
 * A struct that represents the motions that can be requested from the robot.
 */
STREAMABLE(MotionRequest,
{
  ENUM(Motion,
  {,
    walk,
    kick,
    specialAction,
    stand,
    getUp,
    fall,
  });

  /** Draws something*/
  void draw() const,

  (Motion)(specialAction) motion, /**< The selected motion. */
  (SpecialActionRequest) specialActionRequest, /**< The special action request, if it is the selected motion. */
  (WalkRequest) walkRequest, /**< The walk request, if it is the selected motion. */
  (KickRequest) kickRequest, /**< The kick request, if it is the selected motion. */
});
