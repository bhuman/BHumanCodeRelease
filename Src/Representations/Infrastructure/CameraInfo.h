/**
 * @file CameraInfo.h
 *
 * Declaration of struct CameraInfo
 *
 * @author Matthias Jüngel
 * @author Walter Nistico
 */

#pragma once

#include "Streaming/Enum.h"
#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * Information about the camera which provides the images for the robot
 */
STREAMABLE(CameraInfo,
{
  /**
   * Intrinsic camera parameters: axis skew is modeled as 0 (90° perfectly orthogonal XY)
   * and the same has been modeled for focal axis aspect ratio; distortion is considering
   * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
   */
  float focalLength = 0;
  float focalLengthInv = 0; // (1/focalLength) used to speed up certain calculations
  float focalLenPow2 = 0;
  float focalLengthHeight = 0;
  float focalLengthHeightInv = 0;

  void updateFocalLength();

  void draw() const;

  void onRead() { updateFocalLength(); },

  (int)(0) width,
  (int)(0) height,
  (int)(30) frameRate,
  (Angle) openingAngleWidth,
  (Angle) openingAngleHeight,
  (Vector2f) opticalCenter,
});
