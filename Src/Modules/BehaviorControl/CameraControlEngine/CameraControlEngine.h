/**
 * @file CameraControlEngine.h
 * To execute the current HeadMotionRequest the CameraControlEngine
 * chooses one of the two cameras of the Nao and computes the appropriate
 * angles for the head joints to execute the current HeadMotionRequest.
 * @author Andreas Stolpmann
 * (Based on an old version by Felix Wenk)
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(CameraControlEngine,
{,
  REQUIRES(RobotDimensions),
  REQUIRES(CameraCalibration),
  REQUIRES(HeadLimits),
  REQUIRES(HeadMotionRequest),
  REQUIRES(RobotCameraMatrix),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(HeadAngleRequest),
  LOADS_PARAMETERS(
  {,
    (Angle) moveHeadThreshold,
    (Angle) defaultTilt,
  }),
});

class CameraControlEngine : public CameraControlEngineBase
{
public:
  CameraControlEngine();

private:
  Rangea panBounds;

  void update(HeadAngleRequest& headAngleRequest) override;

  void calculatePanTiltAngles(const Vector3f& hip2Target, CameraInfo::Camera camera, Vector2a& panTilt) const;
  void adjustTiltBoundToShoulder(Angle pan, CameraInfo::Camera camera, Rangea& tiltBound) const;
};
