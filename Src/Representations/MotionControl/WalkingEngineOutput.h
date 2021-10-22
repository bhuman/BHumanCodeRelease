/**
 * @file WalkingEngineOutput.h
 *
 * This file declares a representation that contains the static output of the walking engine.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(WalkingEngineOutput,
{,
  (Pose2f)(45_deg, 100.f, 100.f) maxSpeed, /**< The maximum speed possible. */
  (float)(100.f) maxSpeedBackwards, /**< The maximum backward speed possible. */
  (float)(0.25f) walkStepDuration, /**< Duration of one step in seconds. */
});
