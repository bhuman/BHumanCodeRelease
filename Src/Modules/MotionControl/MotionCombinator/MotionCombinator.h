/**
 * @file Modules/MotionControl/MotionCombinator.h
 * This file declares a module that combines the motions created by the different modules.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/MotionUtilities.h"

MODULE(MotionCombinator,
{,
  REQUIRES(ArmJointRequest),
  REQUIRES(ArmMotionSelection),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FallDownState),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(KeyStates),
  REQUIRES(KickEngineOutput),
  REQUIRES(LegJointRequest),
  REQUIRES(HeadJointRequest),
  REQUIRES(LegMotionSelection),
  REQUIRES(RobotInfo),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(JointRequest),
  REQUIRES(JointRequest),
  PROVIDES(MotionInfo),
  PROVIDES(OdometryData),
  LOADS_PARAMETERS(
  {,
    (unsigned) recoveryTime, /**< The number of frames to interpolate after emergency-stop. */
    (bool) useStiffnessDirectly,
    (bool) debugArms,
    (bool) useAccFusion,
    (Vector3f) dynamicVariance,
    (Vector2f) measurementVariance,
    (float) pseudoVariance,
    (bool) useFilteredAcc,
  }),
});

class MotionCombinator : public MotionCombinatorBase
{
private:
  OdometryData odometryData; /**< The odometry data. */
  MotionInfo motionInfo; /**< Information about the motion currently executed. */
  Pose2f specialActionOdometry; /**< Workaround for accumulating special action odometry. */

  OdometryData lastOdometryData;
  JointRequest lastJointRequest;

  UKF<3> odometryUKF = UKF<3>(Vector3f::Zero()); /**< Estimates the speed of the robot based on walking engine output and accelerometer */

  void update(JointRequest& jointRequest) override;
  void update(OdometryData& odometryData) override;
  void update(MotionInfo& motionInfo) override { motionInfo = this->motionInfo; }

  void applyDynamicStiffness(JointRequest& jointRequest) const;

  void applyDamageConfig(JointRequest& jointRequest) const;
  void debugReleaseArms(JointRequest& jointRequest) const;

  void estimateOdometryOffset(Vector2f& offset);
};
