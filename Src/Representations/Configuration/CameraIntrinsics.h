/**
 * @file CameraIntrinsics.h
 * Declaration of a struct for representing the intrinsic parameters of the cameras.
 * @author Alexis Tsogias
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * This struct stores the opening angles and optical centers of both cameras.
 * The optical center is stored in relative coordinates with a range from 0 to 1.
 * Restoring the optical center in pixels is a matter of multiplying the width / height of the cameras.
 * Thus, only resolutions with the same aspect ratios should be used.
 */
STREAMABLE(CameraIntrinsics,
{
  STREAMABLE(Camera,
  {,
    (Angle) openingAngleWidth,
    (Angle) openingAngleHeight,
    (Vector2f) opticalCenter,
  }),

  (ENUM_INDEXED_ARRAY(Camera, CameraInfo::Camera)) cameras,
});
