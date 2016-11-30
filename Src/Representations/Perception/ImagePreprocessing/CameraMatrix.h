/**
 * @file CameraMatrix.h
 * Declaration of CameraMatrix and RobotCameraMatrix representation.
 * @author Colin Graf
 */

#pragma once

#include "Tools/Math/Pose3f.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"

/**
 * Matrix describing transformation from center of hip to camera.
 */
struct RobotCameraMatrix : public Pose3f
{
  RobotCameraMatrix() = default;
  RobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera);

  /** Draws the camera matrix. */
  void draw() const;

  void computeRobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera);
};

/**
 * Matrix describing transformation from ground (center between booth feet) to camera.
 */
STREAMABLE_WITH_BASE(CameraMatrix, Pose3f,
{
  CameraMatrix() = default;

  /**
   * Kind of copy-constructor.
   * @param pose The other pose.
   */
  CameraMatrix(const Pose3f& pose);
  CameraMatrix(const Pose3f& torsoMatrix, const Pose3f& robotCameraMatrix, const CameraCalibration& cameraCalibration);

  void computeCameraMatrix(const Pose3f& torsoMatrix, const Pose3f& robotCameraMatrix, const CameraCalibration& cameraCalibration);

  /** Draws the camera matrix. */
  void draw() const,

  (bool)(true) isValid, /**< Matrix is only valid if motion was stable. */
});
