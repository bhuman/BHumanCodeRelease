/**
 * @file CameraMatrixProvider.h
 * This file declares a class to calculate the position of the camera for the Nao.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"

MODULE(CameraMatrixProvider,
{,
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo), // for debug drawing
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions), // for debug drawing
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(MotionInfo),
  REQUIRES(RobotCameraMatrix),
  REQUIRES(RobotModel),
  REQUIRES(RobotInfo),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotPose), // for debug drawing
  PROVIDES(CameraMatrix),
});

class CameraMatrixProvider : public CameraMatrixProviderBase
{
private:
  void update(CameraMatrix& cameraMatrix) override;

  void camera2image(const Vector3f& camera, Vector2f& image) const;
  bool intersectLineWithCullPlane(const Vector3f& lineBase, const Vector3f& lineDir,
                                  Vector3f& point) const;
  void drawFieldLines(const CameraMatrix& cameraMatrix) const;

  STREAMABLE(ModelPoints,
  {,
    (std::vector<float>) thighPoints,
    (std::vector<int>) thighIndex,
    (std::vector<float>) shinePoints,
    (std::vector<int>) shineIndex,
    (std::vector<float>) footPoints,
    (std::vector<int>) footIndex,
  });

  ModelPoints p;

  void drawRobotParts();
};
