/**
 * @file Representations/MotionControl/FallEngineOutput.h
 * A struct that represents the output of the FallEngine.
 * @author Bernd Poppinga
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Math/Pose2f.h"

STREAMABLE_WITH_BASE(FallEngineOutput, JointRequest,
{,
  (Pose2f) odometryOffset,
  (bool)(false) active,
  (unsigned)(0) startTime,
  (bool)(false) fallingBackwards,
  (bool)(false) fallingForward,
  (bool)(false) waitingForGetup,
});
