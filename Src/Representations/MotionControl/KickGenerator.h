/**
 * @file KickGenerator.h
 *
 * This file declares a representation that can generate kick phases.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/MotionControl/KickRequest.h"
#include "Tools/Function.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Streams/AutoStreamable.h"
#include <memory>

STREAMABLE(KickGenerator,
{
  FUNCTION(std::unique_ptr<MotionPhase>(const KickRequest&, const MotionPhase&)) createPhase, /**< Creates a phase for the given motion request. */
});
