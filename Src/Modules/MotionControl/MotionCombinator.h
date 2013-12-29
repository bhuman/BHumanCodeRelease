/**
* @file Modules/MotionControl/MotionCombinator.h
* This file declares a module that combines the motions created by the different modules.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ArmMotionEngineOutput.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/BikeEngineOutput.h"
#include "Representations/MotionControl/IndykickEngineOutput.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/BallTakingOutput.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/HeadJointRequest.h"

MODULE(MotionCombinator)
  REQUIRES(FilteredJointData)
  REQUIRES(FallDownState)
  REQUIRES(MotionSelection)
  REQUIRES(WalkingEngineOutput)
  REQUIRES(BikeEngineOutput)
  REQUIRES(IndykickEngineOutput)
  REQUIRES(SpecialActionsOutput)
  REQUIRES(WalkingEngineStandOutput)
  REQUIRES(GetUpEngineOutput)
  REQUIRES(BallTakingOutput)
  REQUIRES(HeadJointRequest)
  REQUIRES(HardnessSettings)
  REQUIRES(ArmMotionEngineOutput)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(OdometryData)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(JointRequest)
  PROVIDES_WITH_MODIFY(MotionInfo)
  LOADS_PARAMETER(bool, emergencyOffEnabled)
  LOADS_PARAMETER(unsigned, recoveryTime) /**< The number of frames to interpolate after emergency-stop. */
END_MODULE

class MotionCombinator : public MotionCombinatorBase
{
private:
  JointData lastJointData; /**< The measured joint angles the last time when not interpolating. */
  OdometryData odometryData; /**< The odometry data. */
  MotionInfo motionInfo; /**< Information about the motion currently executed. */
  Pose2D specialActionOdometry; /**< workaround for accumulating special action odometry*/

  void update(OdometryData& odometryData);
  void update(JointRequest& jointRequest);
  void update(MotionInfo& motionInfo) {motionInfo = this->motionInfo;}

  void saveFall(JointRequest& JointRequest);
  void centerHead(JointRequest& JointRequest);
  unsigned currentRecoveryTime;

  bool headJawInSavePosition;
  bool headPitchInSavePosition;

#ifndef RELEASE
  OdometryData lastOdometryData;
  JointRequest lastJointRequest;
#endif

  /**
  * The method copies all joint angles from one joint request to another,
  * but only those that should not be ignored.
  * @param source The source joint request. All angles != JointData::ignore will be copied.
  * @param target The target joint request.
  */
  void copy(const JointRequest& source, JointRequest& target) const;

  /**
  * The method interpolates between two joint requests.
  * @param from The first source joint request. This one is the starting point.
  * @param to The second source joint request. This one has to be reached over time.
  * @param fromRatio The ratio of "from" in the target joint request.
  * @param target The target joint request.
  * @param interpolateHardness Whether to interpolate hardness.
  */
  void interpolate(const JointRequest& from, const JointRequest& to, float fromRatio, JointRequest& target, bool interpolateHardness) const;

public:
  /**
  * Default constructor.
  */
  MotionCombinator();
};
