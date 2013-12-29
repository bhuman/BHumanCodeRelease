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
#include "Tools/InverseKinematic.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Perception/BodyContour.h"

MODULE(CameraControlEngine)
  REQUIRES(RobotDimensions)
  REQUIRES(CameraCalibration)
  REQUIRES(CameraInfo)
  REQUIRES(HeadLimits)
  REQUIRES(HeadMotionRequest)
  REQUIRES(RobotCameraMatrix)
  REQUIRES(RobotModel)
  REQUIRES(TorsoMatrix)
  PROVIDES_WITH_MODIFY(HeadAngleRequest)
  LOADS_PARAMETER(float, moveHeadThreshold)
  LOADS_PARAMETER(float, defaultTilt)
END_MODULE

class CameraControlEngine : public CameraControlEngineBase
{
public:
  CameraControlEngine();

private:
  float maxPan, minPan;

  void update(HeadAngleRequest& headAngleRequest);

  void calculatePanTiltAngles(const Vector3<>& hip2Target, bool lowerCamera, Vector2<>& panTilt) const;
  void adjustTiltBoundToShoulder(const float pan, const bool lowerCamera, Vector2<>& tiltBound) const;
};