/**
 * @file Representations/Sensing/FallDownState.h
 *
 * Declaration of class FallDownState
 *
 * @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
 * @class FallDownState
 *
 * A class that represents the current state of the robot's body
 */
STREAMABLE(FallDownState,
{
public:
  ENUM(State,
    undefined,
    upright,
    onGround,
    staggering,
    falling
  );

  ENUM(Direction,
    none,
    front,
    left,
    back,
    right
  );

  ENUM(Sidestate,
    noot, // since "not" is already a keyword...
    leftwards,
    rightwards,
    fallen // robot did not get up since last sideward fall
  );

  /** Debug drawing. */
  void draw() const,

  (State)(undefined) state, /**< Current state of the robot's body. */
  (Direction)(none) direction, /**< The robot is falling / fell into this direction. */
  (Sidestate)(noot) sidewards, /**< Did the robot fell sidewards before? */
  (float)(0) odometryRotationOffset,
});
