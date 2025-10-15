/**
 * @file BoosterSolePressureStateProvider.h
 * This modules provides fake FSR data based on the kinematic and estimated torso orientation
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"

MODULE(BoosterSolePressureStateProvider,
{,
  REQUIRES(JointSensorData),
  REQUIRES(MassCalibration),
  USES(MotionInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(FsrSensorData),
  DEFINES_PARAMETERS(
  {,
    (Rangef)(0.f, 5.f) removeGroundDistanceByTorque,
    (Rangef)(10.f, 20.f) walkTorqueGroundContactScaleRange,
    (Rangef)(5.f, 15.f) standTorqueGroundContactScaleRange,
    (Rangef)(0.25f, 0.5f) torqueGroundContactPercentRange,
    (float)(4.f) maxDistanceToGround,
    (Rangef)(-2.f, 2.f) ankleTorqueClipRange,
  }),
});

class BoosterSolePressureStateProvider : public BoosterSolePressureStateProviderBase
{
  static constexpr float torqueConversion = 0.01f;

  struct FSRPoint
  {
    Vector3f point;
    FsrSensors::FsrSensor sensor;
    Legs::Leg leg;
  };

  void update(FsrSensorData& theFsrSensorData) override;
};
