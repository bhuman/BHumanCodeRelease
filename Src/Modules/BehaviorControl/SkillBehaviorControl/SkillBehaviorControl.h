/**
 * @file SkillBehaviorControl.h
 *
 * This file declares the behavior control for the skill layer.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Random.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStateInformation.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/ClearTarget.h"
#include "Representations/BehaviorControl/DribbleTarget.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/InterceptBallInformation.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/IllegalAreas.h"
#include "Representations/BehaviorControl/KickAtGoalEvaluation.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibLookActive.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/BehaviorControl/PassTargetEvaluation.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JoystickState.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/EmergencyButtonState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Modeling/BallLostModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Perception/ObstaclePercepts/RobotPercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/BehaviorControl/HeadOrientation.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Tools/Motion/ReduceWalkSpeedType.h"
#include "Tools/BehaviorControl/SideWalkingRequest.h"
#include "Debugging/Annotation.h"
#include "Framework/Module.h"
#include "Platform/SystemCall.h"
#include <regex> // not needed in header, but would otherwise be broken by CABSL
#include "Tools/BehaviorControl/Cabsl.h"

#ifndef FULL_HEADER
#include "SpeedUp.h"
#endif

class SkillBehaviorControl;

MODULE(SkillBehaviorControl,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(ArmMotionInfo),
  REQUIRES(BallLostModel),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraCalibrationStatus),
  REQUIRES(CameraInfo),
  REQUIRES(ClearTarget),
  REQUIRES(DribbleTarget),
  REQUIRES(EmergencyButtonState),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(ExpectedGoals),
  REQUIRES(ExtendedGameState),
  REQUIRES(FallDownState),
  REQUIRES(FieldBall),
  REQUIRES(FieldInterceptBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldRating),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(GlobalOpponentsModel),
  REQUIRES(HeadLimits),
  REQUIRES(HeadMotionInfo),
  REQUIRES(IllegalAreas),
  REQUIRES(IndirectKick),
  REQUIRES(InertialData),
  REQUIRES(InterceptBallInformation),
  REQUIRES(JointAngles),
  REQUIRES(JoystickState),
  REQUIRES(KeyStates),
  REQUIRES(KickAtGoalEvaluation),
  REQUIRES(KickInfo),
  REQUIRES(LibCheck),
  REQUIRES(LibLookActive),
  REQUIRES(LibPosition),
  REQUIRES(LibWalk),
  REQUIRES(MotionInfo),
  REQUIRES(RobotPercept),
  REQUIRES(ObstacleModel),
  REQUIRES(OdometryData),
  REQUIRES(PassEvaluation),
  REQUIRES(PassTargetEvaluation),
  REQUIRES(PathPlanner),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotHealth),
  REQUIRES(RobotPose),
  REQUIRES(SkillRequest),
  REQUIRES(StrategyStatus),
  REQUIRES(SystemSensorData),
  REQUIRES(TeamData),
  REQUIRES(TeamBallModel),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(ActivationGraph),
  REQUIRES(ActivationGraph),
  PROVIDES(ArmMotionRequest),
  PROVIDES(BehaviorStateInformation),
  PROVIDES(BehaviorStatus),
  PROVIDES(CalibrationRequest),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),
  LOADS_PARAMETERS(
  {,
    (cabsl::Cabsl<SkillBehaviorControl>::OptionInfos::Option) root, /**< The root option. */
    (int) continueReceivePassTime, /**< If the teammate no longer communicates a pass, continue receive pass for this time. */
    (int) ignoreReceivePassAfterTime, /**< If the communicated pass is too old, ignore it. */
    (Pose2f) maxWalkSpeedAtFieldBorder, /**< Max walk speed outside of the field. */
    (float) outSideFieldBorderRange, /**< Scale down walk speed over this distance outside the field. */
    (float) fieldBorderOffset, /**< Offset to field border to start scaling down speed. */
  }),
});

class SkillBehaviorControl : public SkillBehaviorControlBase, public cabsl::Cabsl<SkillBehaviorControl>
{
public:
  /** Constructor. */
  SkillBehaviorControl();

private:
  /**
   * Updates the activation graph.
   * @param activationGraph The provided activation graph.
   */
  void update(ActivationGraph& activationGraph) override;

  /**
   * Updates the arm motion request.
   * @param armMotionRequest The provided arm motion request.
   */
  void update(ArmMotionRequest& armMotionRequest) override { armMotionRequest = theArmMotionRequest; }

  /**
   * Updates the behavior state information.
   * @param behaviorStateInformation The provided behavior state information.
   */
  void update(BehaviorStateInformation& behaviorStateInformation) override
  {
    behaviorStateInformation = theBehaviorStateInformation;
  };

  /**
   * Updates the behavior status.
   * @param behaviorStatus The provided behavior status.
   */
  void update(BehaviorStatus& behaviorStatus) override { behaviorStatus = theBehaviorStatus; }

  /**
   * Updates the camera calibration request.
   * @param cameraCalibrationRequest The provided camera calibration request.
   */
  void update(CalibrationRequest& calibrationRequest) override { calibrationRequest = theCalibrationRequest; }

  /**
   * Updates the head motion request.
   * @param headMotionRequest The provided head motion request.
   */
  void update(HeadMotionRequest& headMotionRequest) override { headMotionRequest = theHeadMotionRequest; }

  /**
   * Updates the motion request.
   * @param motionRequest The provided motion request.
   */
  void update(MotionRequest& motionRequest) override { motionRequest = theMotionRequest; }

  unsigned lastReceivePassRequestTimestamp = 0;
  int receivePassPlayerNumber = -1;
  bool isSlowingDownLookActive = false;

protected:
  /** Executes the skill request. */
  void executeRequest();

  /**< Clips walk speed near field border. */
  void handleWalkSpeedAtFieldBorder();

  unsigned timeWhenAnnouncedEmptySkillRequest = 0; /**< The last time when the robot said that its skill request is empty. */
  unsigned lastBallOutOfFieldWaitTimestamp = 0; /**< The last time HandleBallOutOfField was active. */

  ArmMotionRequest theArmMotionRequest; /**< The arm motion request that is modified by the behavior. */
  BehaviorStateInformation theBehaviorStateInformation; /**< Information about the state transitions of the behavior. */
  BehaviorStatus theBehaviorStatus; /**< The behavior status that is modified by the behavior. */
  CalibrationRequest theCalibrationRequest; /**< The camera calibration request that is modified by the behavior. */
  HeadMotionRequest theHeadMotionRequest; /**< The head motion request that is modified by the behavior. */
  MotionRequest theMotionRequest; /**< The motion request that is modified by the behavior. */

#include "Options/Options.h"
#include "Skills/Arms/Arms.h"
#include "Skills/Ball/Ball.h"
#include "Skills/Calibration/Calibration.h"
#include "Skills/Demo/Demo.h"
#include "Skills/Head/Head.h"
#include "Skills/Output/Output.h"
#include "Skills/Support/Support.h"
#include "Skills/Walk/Walk.h"
};
