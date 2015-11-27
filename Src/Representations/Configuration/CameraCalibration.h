/**
 * @file CameraCalibration.h
 * Declaration of a struct for representing the calibration values of the camera.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"
#include "Tools/Module/Next.h"

STREAMABLE(CameraCalibration,
{,
  (Vector3a) lowerCameraRotationCorrection, //!< The correction of the lower camera rotation
  (Vector3a) upperCameraRotationCorrection, //!< The correction of the upper camera rotation
  (Vector2a) bodyRotationCorrection, //!< The correction of the body rotation
});

using CameraCalibrationNext = Next<CameraCalibration>;
