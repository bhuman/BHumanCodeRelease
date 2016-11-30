/**
 * @file Modules/MotionControl/MotionCombinator.h
 * This file declares a module that combines the motions created by the different modules.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"

MODULE(MotionCombinator,
{,
  REQUIRES(ArmJointRequest),
  REQUIRES(ArmMotionSelection),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
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
    (bool) emergencyOffEnabled,
    (unsigned) recoveryTime, /**< The number of frames to interpolate after emergency-stop. */
    (bool) debugArms,
  }),
});

class MotionCombinator : public MotionCombinatorBase
{
private:
  OdometryData odometryData; /**< The odometry data. */
  MotionInfo motionInfo; /**< Information about the motion currently executed. */
  Pose2f specialActionOdometry; /**< workaround for accumulating special action odometry. */

  unsigned currentRecoveryTime;

  bool headYawInSafePosition = false;
  bool headPitchInSafePosition = false;

  OdometryData lastOdometryData;
  JointRequest lastJointRequest;

public:
  MotionCombinator() : currentRecoveryTime(recoveryTime + 1) {}

  /**
   * The method copies all joint angles from one joint request to another,
   * but only those that should not be ignored.
   * @param source The source joint request. All angles != JointAngles::ignore will be copied.
   * @param target The target joint request.
   */
  void static copy(const JointRequest& source, JointRequest& target,
                   const StiffnessSettings& theStiffnessSettings,
                   const Joints::Joint startJoint, const Joints::Joint endJoint);

  /**
   * The method interpolates between two joint requests.
   * @param from The first source joint request. This one is the starting point.
   * @param to The second source joint request. This one has to be reached over time.
   * @param fromRatio The ratio of "from" in the target joint request.
   * @param target The target joint request.
   * @param interpolateStiffness Whether to interpolate stiffness.
   */
  void static interpolate(const JointRequest& from, const JointRequest& to, float fromRatio, JointRequest& target, bool interpolateStiffness,
                          const StiffnessSettings& theStiffnessSettings, const JointAngles& lastJointAngles,
                          const Joints::Joint startJoint, const Joints::Joint endJoint);

private:
  void update(JointRequest& jointRequest);
  void update(OdometryData& odometryData);
  void update(MotionInfo& motionInfo) { motionInfo = this->motionInfo; }

  void safeFall(JointRequest& JointRequest);
  void centerHead(JointRequest& JointRequest);

  void eraseStiffness(JointRequest& jointRequest);
  void debugReleaseArms(JointRequest& jointRequest);
};
