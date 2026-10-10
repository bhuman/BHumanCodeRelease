/**
 * @file CameraCalibration.h
 * Declaration of a struct for representing the calibration values of the camera.
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Eigen.h"
#include "Math/Angle.h"
#include "Framework/Next.h"

STREAMABLE(CameraCalibration,
{,
  (Vector3a) cameraRotationCorrection, //!< The correction of the both camera rotations
  (Vector2a) bodyRotationCorrection, //!< The correction of the body rotation
});

using CameraCalibrationNext = Next<CameraCalibration>;
