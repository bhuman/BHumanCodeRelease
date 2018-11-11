/**
 * @file CameraInfo.h
 *
 * Declaration of struct CameraInfo
 *
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 * @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * Information about the camera which provides the images for the robot
 */
STREAMABLE(CameraInfo,
{
  /**
   * @enum Camera
   * Enum representing the possible sources of an image.
   */
  ENUM(Camera,
  {,
    upper,
    lower,
  });

  /**
   * Intrinsic camera parameters: axis skew is modelled as 0 (90° perfectly orthogonal XY)
   * and the same has been modeled for focal axis aspect ratio; distortion is considering
   * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
   */
  float focalLength;
  float focalLengthInv; // (1/focalLength) used to speed up certain calculations
  float focalLenPow2;
  float focalLengthHeight;
  float focalLengthHeightInv;

  CameraInfo() = default;
  CameraInfo(Camera camera) : camera(camera) {}

  void updateFocalLength();

  void draw() const;

  void onRead() { updateFocalLength(); },

  (Camera) camera,
  (int) width,
  (int) height,
  (Angle) openingAngleWidth,
  (Angle) openingAngleHeight,
  (Vector2f) opticalCenter,
});
