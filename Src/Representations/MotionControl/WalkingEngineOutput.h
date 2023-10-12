/**
 * @file WalkingEngineOutput.h
 *
 * This file declares a representation that contains the static output of the walking engine.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(WalkingEngineOutput,
{,
  (Pose2f)(45_deg, 100.f, 100.f) maxSpeed, /**< The maximum speed possible. */
  (float)(100.f) maxSpeedBackwards, /**< The maximum backward speed possible. */
  (float)(0.25f) walkStepDuration, /**< Duration of one step in seconds. */

  (Pose2f)(45_deg, 50.f, 50.f) maxStepSize, /**< The maximum step size. */
  (float)(30.f) maxBackwardStepSize, /**< The maximum backward speed possible. */
  (float)(0.25f) stepDuration, /**< Step duration (in s) */

  (Pose2f)(45_deg, 50.f, 50.f) maxPossibleStepSize, /**< The maximum step size that is theoretical possible. DO NOT use for walking. */
  (float)(30.f) maxPossibleBackwardStepSize, /**< The maximum backward speed that is theoretical possible. DO NOT use for walking. */
});
