/**
 * @file LibLookActive.h
 *
 * This file defines a representation that provides helpful methods for controlling look behavior
 *
 * @author Daniel Krause
 */

#pragma once

#include "Tools/Function.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(HeadTarget,
{,
  (Angle) pan,
  (Angle) tilt,
  (Angle) speed,
  (HeadMotionRequest::CameraControlMode) cameraControlMode,
  (bool) stopAndGoMode,
});

STREAMABLE(LibLookActive,
{
  FUNCTION(HeadTarget(const bool withBall, const bool ignoreBall, const bool onlyOwnBall, const bool fixTilt)) calculateHeadTarget,
});
