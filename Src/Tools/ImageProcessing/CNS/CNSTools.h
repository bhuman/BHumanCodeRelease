/**
 * @file CNSTools.h
 *
 * This file declares tools to translate between B-Human data types and
 * data types used in the contour detector implementation.
 *
 * @author Thomas Röfer
 */

#pragma once
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/ImageProcessing/CNS/CameraModelOpenCV.h"
#include "Tools/Math/Pose3f.h"

namespace CNS
{
  /**
   * Converts a 4x4 matrix to a 3D-pose.
   * @param m The 4x4 matrix (top left 3x3 entries are rotation, forth column is position).
   * @return The corresponding 3D-pose.
   */
  Pose3f toPose3f(const Matrix4d& m);

  /**
   * Converts a 3D-pose to a 4x4 matrix.
   * @param p The corresponding 3D-pose.
   * @return The 4x4 matrix (top left 3x3 entries are rotation, forth column is position).
   */
  Matrix4d toMatrix4d(const Pose3f& p);

  /**
   * Creates an OpenCV camera model.
   * @param cameraInfo The information about the camera.
   * @param cameraIntrinsics The calibration of the camera.
   * @return The corresponding OpenCV camera model.
   */
  CameraModelOpenCV toCameraModelOpenCV(const CameraInfo& cameraInfo, const CameraIntrinsics& cameraIntrinsics);
}
