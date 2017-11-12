/**
 * @file BehaviorControl.h
 * Declaration of the C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/SPLStandardBehaviorStatus.h"
#include "Representations/BehaviorControl/TimeToReachBall.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/CameraCalibrationRating.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"
#include <algorithm>
#include <limits>
#include <sstream>
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX BehaviorControl::
#endif
#include "Tools/Cabsl.h" // include last, because macros might mix up other header files

MODULE(BehaviorControl,
{,
  REQUIRES(ActivationGraph),
  REQUIRES(ArmContactModel),
  REQUIRES(ArmKeyFrameEngineOutput),
  REQUIRES(ArmMotionInfo),
  REQUIRES(ArmMotionSelection),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters),
  REQUIRES(CameraCalibrationRating),
  REQUIRES(CameraStatus),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FallDownState),
  REQUIRES(FieldDimensions),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(GlobalFieldCoverage),
  REQUIRES(GroundContactState),
  REQUIRES(HeadMotionEngineOutput),
  REQUIRES(JointAngles),
  REQUIRES(KeyStates),
  REQUIRES(KickEngineOutput),
  REQUIRES(LibDemo),
  REQUIRES(LibCodeRelease),
  REQUIRES(MotionInfo),
  REQUIRES(WalkGenerator),
  REQUIRES(LegMotionSelection),
  REQUIRES(ObstacleModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(PathPlanner),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(SideConfidence),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(ActivationGraph),
  PROVIDES(ArmMotionRequest),
  PROVIDES(BehaviorStatus),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),
  PROVIDES(SPLStandardBehaviorStatus),
});

/**
 * @class BehaviorControl
 * A C-based state machine behavior control module.
 */
class BehaviorControl : public BehaviorControlBase, public Cabsl<BehaviorControl>
{
#include "Options.h"

  STREAMABLE(Parameters,
  {
    /** Helper for streaming a vector of enums that are defined in another class. */
    struct OptionInfos : Cabsl<BehaviorControl>::OptionInfos {using Options = std::vector<Option>;},

    ((OptionInfos) Options) roots, /**< All options that function as behavior roots. */
  });

  void update(ActivationGraph& activationGraph);

  /** Updates the arm motion request by copying it from the behavior */
  void update(ArmMotionRequest& armMotionRequest) { armMotionRequest = theArmMotionRequest; }

  /** Update the behavior status by copying it from the behavior */
  void update(BehaviorStatus& behaviorStatus) {behaviorStatus = theBehaviorStatus;}

  /** Updates the head motion request by copying it from the behavior */
  void update(HeadMotionRequest& headMotionRequest) {headMotionRequest = theHeadMotionRequest;}

  /** Updates the motion request by copying it from the behavior */
  void update(MotionRequest& motionRequest) {motionRequest = theMotionRequest;}

  /** Updates the standard behavior status by copying it from the behavior */
  void update(SPLStandardBehaviorStatus& splStandardBehaviorStatus) {splStandardBehaviorStatus = theSPLStandardBehaviorStatus;}

  Parameters parameters; /**< The root options. */
  BehaviorStatus theBehaviorStatus;
  HeadMotionRequest theHeadMotionRequest; /**< The head motion request that will be set. */
  ArmMotionRequest theArmMotionRequest;
  MotionRequest theMotionRequest;
  SPLStandardBehaviorStatus theSPLStandardBehaviorStatus;

public:
  BehaviorControl();
};
