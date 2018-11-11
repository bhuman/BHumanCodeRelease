/**
 * @file CameraIntrinsics.h
 * Declaration of a struct for representing the intrinsic parameters of the cameras.
 * @author Alexis Tsogias
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"

/**
 * This struct stores the opening angles and optical centers of both cameras.
 * The optical center is stored in relative coordinates with a range from 0 to 1.
 * Restoring the optical center in pixels is a matter of multiplying the width / height of the cameras.
 * Thus, only resolutions with the same aspect ratios should be used.
 */
STREAMABLE(CameraIntrinsics,
{,
  (Angle) upperOpeningAngleWidth,
  (Angle) upperOpeningAngleHeight,
  (Vector2f) upperOpticalCenter,
  (Angle) lowerOpeningAngleWidth,
  (Angle) lowerOpeningAngleHeight,
  (Vector2f) lowerOpticalCenter,
});
