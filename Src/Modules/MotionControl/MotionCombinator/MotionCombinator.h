/**
 * @file Modules/MotionControl/MotionCombinator.h
 * This file declares a module that combines the motions created by the different modules.
 * @author Thomas Röfer
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FilteredCurrent.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/RingBuffer.h"

MODULE(MotionCombinator,
{,
  REQUIRES(ArmJointRequest),
  REQUIRES(ArmMotionSelection),
  REQUIRES(DamageConfigurationBody),
  USES(EnergySaving),
  REQUIRES(FallDownState),
  REQUIRES(FilteredCurrent),
  REQUIRES(FrameInfo),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(HeadJointRequest),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  REQUIRES(KeyStates),
  REQUIRES(KickEngineOutput),
  REQUIRES(LegJointRequest),
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
    (Angle) maxJointAcc, /**< Maximum speed change in angle/s^2. */
  }),
});

class MotionCombinator : public MotionCombinatorBase
{
private:
  OdometryData odometryData; /**< The odometry data. */
  MotionInfo motionInfo; /**< Information about the motion currently executed. */
  Pose2f specialActionOdometry; /**< Workaround for accumulating special action odometry. */

  OdometryData lastOdometryData;
  RingBuffer<Angle, 2> lastHYPRequest; /**< The last two joint requests sent for the HYP. */

  UKF<3> odometryUKF = UKF<3>(Vector3f::Zero()); /**< Estimates the speed of the robot based on walking engine output and accelerometer */

  void update(JointRequest& jointRequest) override;
  void update(OdometryData& odometryData) override;
  void update(MotionInfo& motionInfo) override { motionInfo = this->motionInfo; }

  void applyDynamicStiffness(JointRequest& jointRequest) const;

  void applyDamageConfig(JointRequest& jointRequest) const;
  void debugReleaseArms(JointRequest& jointRequest) const;

  void estimateOdometryOffset(Vector2f& offset);

  void applyStandHeat(JointRequest& jointRequest); /**< If the Robot is standing then adjust the leg joints to prevent overheating */
};
