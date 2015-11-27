/**
 * @file Modules/MotionControl/MotionCombinator.h
 * This file declares a module that combines the motions created by the different modules.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author (arm upgrade) Jesse Richter-Klug
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/DmpKickEngineOutput.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"

MODULE(MotionCombinator,
{,
  REQUIRES(ArmKeyFrameEngineOutput),
  REQUIRES(ArmMotionSelection),
  REQUIRES(DmpKickEngineOutput),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(HeadJointRequest),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(KickEngineOutput),
  REQUIRES(MotionSelection),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(RobotInfo),
  REQUIRES(StandOutput),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(JointRequest),
  REQUIRES(JointRequest),
  PROVIDES(ArmMotionInfo),
  PROVIDES(MotionInfo),
  PROVIDES(OdometryData),
  LOADS_PARAMETERS(
  {,
    (bool) emergencyOffEnabled,
    (unsigned) recoveryTime, /**< The number of frames to interpolate after emergency-stop. */
  }),
});

class MotionCombinator : public MotionCombinatorBase
{
private:
  NonArmeMotionEngineOutput theNonArmeMotionEngineOutput;

  JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */
  OdometryData odometryData; /**< The odometry data. */
  MotionInfo motionInfo; /**< Information about the motion currently executed. */
  ArmMotionInfo armMotionInfo; /**< Information about the arm motion currently executed. */
  Pose2f specialActionOdometry; /**< workaround for accumulating special action odometry. */

  unsigned currentRecoveryTime;

  bool headJawInSavePosition;
  bool headPitchInSavePosition;
  bool isFallingStarted;
  unsigned fallingFrame;

  OdometryData lastOdometryData;
  JointRequest lastJointRequest;

public:
  /**
  * Default constructor.
  */
  MotionCombinator();

private:
  void update(OdometryData& odometryData);
  void update(JointRequest& jointRequest);
  void update(MotionInfo& motionInfo) { motionInfo = this->motionInfo; }
  void update(ArmMotionInfo& armMotionInfo) { armMotionInfo = this->armMotionInfo; }

  void saveFall(JointRequest& JointRequest);
  void centerHead(JointRequest& JointRequest);
  void centerArms(JointRequest& jointRequest);
  void centerArm(JointRequest& jointRequest, bool left);

  /**
   * The method copies all joint angles from one joint request to another,
   * but only those that should not be ignored.
   * @param source The source joint request. All angles != JointAngles::ignore will be copied.
   * @param target The target joint request.
   */
  void copy(const JointRequest& source, JointRequest& target,
            const Joints::Joint startJoint = static_cast<Joints::Joint>(0),
            const Joints::Joint endJoint = static_cast<Joints::Joint>(Joints::numOfJoints - 1)) const;

  /**
   * The method interpolates between two joint requests.
   * @param from The first source joint request. This one is the starting point.
   * @param to The second source joint request. This one has to be reached over time.
   * @param fromRatio The ratio of "from" in the target joint request.
   * @param target The target joint request.
   * @param interpolateStiffness Whether to interpolate stiffness.
   */
  void interpolate(const JointRequest& from, const JointRequest& to, float fromRatio, JointRequest& target, bool interpolateStiffness,
                   const Joints::Joint startJoint = static_cast<Joints::Joint>(0),
                   const Joints::Joint endJoint = static_cast<Joints::Joint>(Joints::numOfJoints - 1)) const;
};
