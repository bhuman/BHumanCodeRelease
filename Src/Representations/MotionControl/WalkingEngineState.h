#pragma once

#include "Modules/MotionControl/WalkingEngine/WalkingEngineUtils.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(WalkingEngineState,
{,
  (WalkingEngineOutput) unfinishedOutput,
  (WalkingEngineUtils::LegPosture) targetLegPosture,
  (RotationMatrix) standBodyRotation,
  (float) observerMeasurementDelay,
});
