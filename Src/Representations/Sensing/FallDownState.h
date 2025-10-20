/**
 * @file Representations/Sensing/FallDownState.h
 *
 * Declaration of struct FallDownState
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

/**
 * @struct FallDownState
 *
 * A struct that represents the current state of the robot's body
 */
STREAMABLE(FallDownState,
{
  ENUM(State,
  {,
    pickedUp,
    upright,
    staggering,
    falling,
    fallen,
    squatting,
  });

  ENUM(Direction,
  {,
    none,
    front,
    left,
    back,
    right,
  });

  /** Debug drawing. */
  void draw() const,

  (State)(pickedUp) state, /**< Current state of the robot's body. */
  (Direction)(none) direction, /**< The robot is falling / fell into this direction. */
  (float)(0) odometryRotationOffset,
  (unsigned)(0) timestampSinceStateSwitch, /**< The timestamp from when the state switched the last time. */
  (Vector3f)(Vector3f::Zero()) predictedCom,
});
