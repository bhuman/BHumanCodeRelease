/**
 * @file CameraIntrinsics.h
 * Declaration of a struct for representing the intrinsic parameters of the camera.
 * @author Alexis Tsogias
 */

#pragma once

#include "Math/Angle.h"
#include "Math/Eigen.h"

/**
 * This struct stores the opening angles and optical centers of the camera.
 * The optical center is stored in relative coordinates with a range from 0 to 1.
 * Restoring the optical center in pixels is a matter of multiplying the width / height of the camera.
 * Thus, only resolutions with the same aspect ratios should be used.
 */
STREAMABLE(CameraIntrinsics,
{,
  (Angle) openingAngleWidth,
  (Angle) openingAngleHeight,
  (Vector2f) opticalCenter,
});
