/**
 * @file Keypoints.h
 *
 * This file declares a representation that represents the output of the
 * keypoint detector, i.e. pixel positions and confidences of 17 different
 * body parts.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Boundary.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(Keypoints,
{
  /** The indices for the network output. */
  ENUM(Keypoint,
  {,
    nose,
    leftEye,
    rightEye,
    leftEar,
    rightEar,
    leftShoulder,
    rightShoulder,
    leftElbow,
    rightElbow,
    leftWrist,
    rightWrist,
    leftHip,
    rightHip,
    leftKnee,
    rightKnee,
    leftAnkle,
    rightAnkle,
  });

  /** The position of each keypoint and whether it is valid. */
  STREAMABLE(Point,
  {,
    (Vector2f) position,
    (bool)(false) valid,
  });

  /** Draw the keypoints. */
  void draw() const,

  (ENUM_INDEXED_ARRAY(Point, Keypoint)) points, /**< The keypoints in image coordinates. */
  (Boundaryi) patchBoundary, /**< The boundary of the patch that was searched for the keypoints. */
});
