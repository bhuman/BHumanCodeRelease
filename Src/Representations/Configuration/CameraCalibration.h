/**
 * @file CameraCalibration.h
 * Declaration of a struct for representing the calibration values of the camera.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Math/Eigen.h"
#include "Math/Angle.h"
#include "Framework/Next.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(CameraCalibration,
{,
  (ENUM_INDEXED_ARRAY(Vector3a, CameraInfo::Camera)) cameraRotationCorrections, //!< The correction of the both camera rotations
  (Vector2a) bodyRotationCorrection, //!< The correction of the body rotation
});

using CameraCalibrationNext = Next<CameraCalibration>;
