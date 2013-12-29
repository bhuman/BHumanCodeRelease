/**
* @file CameraMatrix.h
* Declaration of CameraMatrix and RobotCameraMatrix representation.
* @author Colin Graf
*/

#pragma once

#include "Tools/Math/Pose3D.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"

/**
* Matrix describing transformation from center of hip to camera.
*/
class RobotCameraMatrix : public Pose3D
{
public:
  /** Draws the camera matrix. */
  void draw() const;

  void computeRobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera);
  RobotCameraMatrix() {}
  RobotCameraMatrix(const RobotDimensions& robotDimensions, const float headYaw, const float headPitch, const CameraCalibration& cameraCalibration, bool upperCamera);
};

/**
* Matrix describing transformation from ground (center between booth feet) to camera.
*/
STREAMABLE_WITH_BASE(CameraMatrix, Pose3D,
{
public:
  /** Kind of copy-constructor.
  * @param pose The other pose.
  */
  CameraMatrix(const Pose3D& pose);

  void computeCameraMatrix(const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration);
  CameraMatrix(const Pose3D& torsoMatrix, const Pose3D& robotCameraMatrix, const CameraCalibration& cameraCalibration);

  /** Draws the camera matrix. */
  void draw() const,

  (bool)(true) isValid, /**< Matrix is only valid if motion was stable. */
});

class CameraMatrixOther : public CameraMatrix {};
class RobotCameraMatrixOther : public RobotCameraMatrix {};
