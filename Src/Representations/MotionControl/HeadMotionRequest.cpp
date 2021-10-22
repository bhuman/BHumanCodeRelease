/**
 * @file Representations/MotionControl/HeadMotionRequest.h
 * This file declares a struct that represents the requested head motion.
 * @author Philip Reichenberg
 */

#include "HeadMotionRequest.h"
#include "Platform/BHAssert.h"

void HeadMotionRequest::verify() const
{
  VERIFY(std::isfinite(pan));
  VERIFY(std::isfinite(tilt));
  VERIFY(std::isfinite(speed));
  VERIFY(std::isfinite(target.x()));
  VERIFY(std::isfinite(target.y()));
  VERIFY(std::isfinite(target.z()));
  ASSERT(mode >= static_cast<Mode>(0) && mode < numOfModes);
  ASSERT(cameraControlMode >= static_cast<CameraControlMode>(0) && cameraControlMode < numOfCameraControlModes);
}
