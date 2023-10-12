/**
 * @file CameraMatrix.h
 * Declaration of CameraMatrix and RobotCameraMatrix representation.
 * @author Colin Graf
 */

#pragma once

#include "Math/SE3fWithCov.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Platform/SystemCall.h"

/**
 * Matrix describing transformation from center of hip to camera.
 */
STREAMABLE_WITH_BASE(RobotCameraMatrix, SE3WithCov,
{
  RobotCameraMatrix() = default;
  RobotCameraMatrix(const RobotDimensions& robotDimensions, const SE3WithCov& head, const CameraCalibration& cameraCalibration, CameraInfo::Camera camera);

  /**< legacy constructor does not incorporate covariance. */
  RobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, CameraInfo::Camera camera);

  /** Draws the camera matrix. */
  void draw() const;

  void computeRobotCameraMatrix(const RobotDimensions& robotDimensions, const SE3WithCov& head, const CameraCalibration& cameraCalibration, CameraInfo::Camera camera),
});

/**
 * Matrix describing transformation from ground (center between booth feet) to camera.
 */
STREAMABLE_WITH_BASE(CameraMatrix, SE3WithCov,
{
private:
  Pose3f invPos; /**< the inverse */

public:
  CameraMatrix() = default;

  /**
   * Kind of copy-constructor.
   * @param pose The other pose.
   */
  CameraMatrix(const Pose3f& pose);
  CameraMatrix(const SE3WithCov& torsoMatrix, const SE3WithCov& robotCameraMatrix, const CameraCalibration& cameraCalibration);

  void computeCameraMatrix(const SE3WithCov& torsoMatrix, const SE3WithCov& robotCameraMatrix, const CameraCalibration& cameraCalibration);

  Pose3f inverse() const
  {
    return invPos;
  }

  void onRead();

  /** Draws the camera matrix. */
  void draw() const,

  (bool)(false) isValid, /**< Matrix is only valid if motion was stable. */
});
