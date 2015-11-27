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
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/CameraMatrix.h"
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
    (float) moveHeadThreshold,
    (float) defaultTilt,
  }),
});

class CameraControlEngine : public CameraControlEngineBase
{
public:
  CameraControlEngine();

private:
  float maxPan, minPan;

  void update(HeadAngleRequest& headAngleRequest);

  void calculatePanTiltAngles(const Vector3f& hip2Target, bool lowerCamera, Vector2f& panTilt) const;
  void adjustTiltBoundToShoulder(const float pan, const bool lowerCamera, Vector2f& tiltBound) const;
};
