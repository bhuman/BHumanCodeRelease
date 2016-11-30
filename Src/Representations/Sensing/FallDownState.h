/**
 * @file Representations/Sensing/FallDownState.h
 *
 * Declaration of struct FallDownState
 *
 * @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct FallDownState
 *
 * A struct that represents the current state of the robot's body
 */
STREAMABLE(FallDownState,
{
  ENUM(State,
  {,
    undefined,
    upright,
    onGround,
    staggering,
    falling,
    pickedUp,
  });

  ENUM(Direction,
  {,
    none,
    front,
    left,
    back,
    right,
  });

  ENUM(Sidestate,
  {,
    noot, // since "not" is already a keyword...
    leftwards,
    rightwards,
    fallen, // robot did not get up since last sideward fall
  });

  ENUM(Provider,
  {,
    fallDownStateDet,
    toppleOverParabolaDet,
  });

  /** Debug drawing. */
  void draw() const,
  
  (State)(undefined) state, /**< Current state of the robot's body. */
  (Direction)(none) direction, /**< The robot is falling / fell into this direction. */
  (Sidestate)(noot) sidewards, /**< Did the robot fell sidewards before? */
  (float)(0) odometryRotationOffset,
  (Provider)(fallDownStateDet) provider,
});
