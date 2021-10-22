/**
 * @file KickoffState.h
 *
 * This file declares a representation of the progress of a kickoff.
 *
 * @authors The authors of \c LibReadyState.h
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(KickoffState,
{,
  (bool)(true) allowedToScore, /**< Whether it is allowed to score goals. */
  (bool)(true) allowedToEnterCenterCircle, /**< Whether it is allowed to enter the center circle. */
});
