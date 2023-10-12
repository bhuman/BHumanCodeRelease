/**
 * @file SkillBehaviorControl.h
 *
 * This file declares the behavior control for the skill layer.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Libs/Math/Random.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/GoaliePose.h"
#include "Representations/BehaviorControl/IllegalAreas.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/RefereeSignal.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/RefereePercept/OptionalImageRequest.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Debugging/Annotation.h"
#include "Framework/Module.h"
#include "Platform/SystemCall.h"
#include <regex> // not needed in header, but would otherwise be broken by CABSL

class SkillBehaviorControl;

#include "Representations/BehaviorControl/Skills.h"
#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX SkillBehaviorControl::
#endif
#include "Tools/Cabsl.h"

MODULE(SkillBehaviorControl,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraStatus),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(ExtendedGameState),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GoaliePose),
  REQUIRES(IllegalAreas),
  REQUIRES(JointSensorData),
  REQUIRES(KickInfo),
  REQUIRES(LibCheck),
  REQUIRES(LibDemo),
  REQUIRES(LibPosition),
  REQUIRES(MotionInfo),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(ObstacleModel),
  REQUIRES(OdometryData),
  REQUIRES(RefereePercept),
  REQUIRES(RobotHealth),
  REQUIRES(RobotPose),
  REQUIRES(SkillRequest),
  REQUIRES(StrategyStatus),
  REQUIRES(TeamData),
  REQUIRES(TeammatesBallModel),
  PROVIDES(ActivationGraph),
  REQUIRES(ActivationGraph),
  PROVIDES(ArmMotionRequest),
  PROVIDES(BehaviorStatus),
  PROVIDES(CalibrationRequest),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),
  PROVIDES(OptionalImageRequest),
  PROVIDES(RefereeSignal),
  LOADS_PARAMETERS(
  {,
    (std::vector<Cabsl<SkillBehaviorControl>::OptionInfos::Option>) options,
    (std::vector<Cabsl<SkillBehaviorControl>::OptionInfos::Option>) playingOptions,
    (bool) useNewHandleCatchBallBehavior,
  }),
});

class SkillBehaviorControl : public SkillBehaviorControlBase, public Cabsl<SkillBehaviorControl>
{
public:
  /** Constructor. */
  SkillBehaviorControl();

  /**
   * Creates extended module info (union of this module's info and requirements of all skills).
   * @return The extended module info.
   */
  static std::vector<ModuleBase::Info> getExtModuleInfo();

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

  /**
   * Updates the optional image request.
   * @param optionalImageRequest The provided optional image request.
   */
  void update(OptionalImageRequest& optionalImageRequest) override { optionalImageRequest = theOptionalImageRequest; }

  /**
   * Updates the referee signal.
   * @param refereeSignal The provided referee signal.
   */
  void update(RefereeSignal& refereeSignal) override { refereeSignal = theRefereeSignal; }

  /** Executes the skill request. */
  void executeRequest();

  unsigned timeWhenAnnouncedEmptySkillRequest = 0; /**< The last time when the robot said that its skill request is empty. */

  ArmMotionRequest theArmMotionRequest; /**< The arm motion request that is modified by the behavior. */
  BehaviorStatus theBehaviorStatus; /**< The behavior status that is modified by the behavior. */
  CalibrationRequest theCalibrationRequest; /**< The camera calibration request that is modified by the behavior. */
  HeadMotionRequest theHeadMotionRequest; /**< The head motion request that is modified by the behavior. */
  MotionRequest theMotionRequest; /**< The motion request that is modified by the behavior. */
  OptionalImageRequest theOptionalImageRequest; /**< The request that decides whether an optional image should be send or not */
  RefereeSignal theRefereeSignal; /**< The referee signal that should be sent to the GameController. */

  SkillRegistry theSkillRegistry; /**< The manager of all skills. */

#include "Representations/BehaviorControl/SkillStubs.h"
#include "Options/PlaySoccer.h"
#include "Options/HandleAnyPlaceDemo.h"
#include "Options/HandleCatchBall.h"
#include "Options/HandleCatchBall2023.h"
#include "Options/HandleGameState.h"
#include "Options/HandleGoalkeeperCatchBall.h"
#include "Options/HandleIllegalAreas.h"
#include "Options/HandlePenaltyKick.h"
#include "Options/HandlePenaltyShootout.h"
#include "Options/HandlePhotoMode.h"
#include "Options/HandlePhysicalRobot.h"
#include "Options/HandlePlayerState.h"
#include "Options/HandleRefereeSignal.h"
#include "Options/HandleReplayWalk.h"
#include "Options/HandleStrikerLostBall.h"
#include "Options/PenaltyShootout/PenaltyKeeper.h"
#include "Options/PenaltyShootout/PenaltyTaker.h"
#include "Options/HandleCameraCalibrationDataCollection.h"
#include "Options/HandleReturnFromSideline.h"
};
