/**
 * @file Representations/MotionControl/StandBodyRotation.h
 * This file declares a struct that represents a body rotation for a stand, with the aim of minimize a difference between the current use of the feet.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct StandBodyRotation
 * A struct that represents a walk requesta bodyrotation while standing.
 */
STREAMABLE(StandBodyRotation,
{,
  (Vector2f)(Vector2f::Zero()) bodyRotation, /**< the body rotation */
});
