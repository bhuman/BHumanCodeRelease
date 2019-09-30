/**
 * @file LogDataProvider.h
 * This file declares a module that provides data replayed from a log file.
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldColors.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Modeling/AlternativeRobotPoseHypothesis.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/LabelImage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SelfLocalizationHypotheses.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/GetUpEngineOutputLog.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGeneratorData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/ImageProcessing/Image.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/TypeInfo.h"
#include <unordered_set>

// No verify when replaying logfiles
#ifndef NDEBUG
#undef _MODULE_VERIFY
#define _MODULE_VERIFY(r) \
  if(SystemCall::getMode() != SystemCall::logFileReplay) \
    ModuleBase::verify(&r);
#endif

MODULE(LogDataProvider,
{,
  USES(CameraInfo),
  PROVIDES_WITHOUT_MODIFY(CameraImage),
  PROVIDES(ActivationGraph),
  PROVIDES(AlternativeRobotPoseHypothesis),
  PROVIDES(AudioData),
  PROVIDES(BallModel),
  PROVIDES(BallPercept),
  PROVIDES(BallSpots),
  PROVIDES(BehaviorStatus),
  PROVIDES(BodyContour),
  PROVIDES(CameraInfo),
  PROVIDES(CameraMatrix),
  PROVIDES(CirclePercept),
  PROVIDES(ECImage),
  PROVIDES(FallDownState),
  PROVIDES(FieldBoundary),
  PROVIDES(FieldColors),
  PROVIDES(FieldLines),
  PROVIDES(FrameInfo),
  PROVIDES(FsrSensorData),
  PROVIDES(GameInfo),
  PROVIDES(GetUpEngineOutputLog),
  PROVIDES(GroundContactState),
  PROVIDES(GroundTruthOdometryData),
  PROVIDES(GroundTruthRobotPose),
  PROVIDES(GroundTruthWorldState),
  PROVIDES(ImageCoordinateSystem),
  PROVIDES(InertialData),
  PROVIDES(InertialSensorData),
  PROVIDES(IntersectionsPercept),
  PROVIDES(JointAngles),
  PROVIDES(JointRequest),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(LabelImage),
  PROVIDES(LinesPercept),
  PROVIDES(MotionInfo),
  PROVIDES(MotionRequest),
  PROVIDES(ObstacleModel),
  PROVIDES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  PROVIDES(Odometer),
  PROVIDES(OdometryData),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(PenaltyMarkPercept),
  PROVIDES(RawGameInfo),
  PROVIDES(RobotHealth),
  PROVIDES(RobotInfo),
  PROVIDES(RobotPose),
  PROVIDES(SelfLocalizationHypotheses),
  PROVIDES(SideConfidence),
  PROVIDES(TeamBallModel),
  PROVIDES(TeamData),
  PROVIDES(TeamPlayersModel),
  PROVIDES(WalkGeneratorData),
  PROVIDES(WalkingEngineOutput),
  PROVIDES(WalkLearner),
  PROVIDES(Whistle),
});

class LogDataProvider : public LogDataProviderBase
{
private:
  static thread_local LogDataProvider* theInstance; /**< Points to the only instance of this class in this thread or is 0 if there is none. */

  ENUM(State,
  {,
    unknown, // No specification available -> replay anyway.
    accept,  // Specification is compatible -> replay.
    convert, // Specification not compatible -> convert.
  });

  std::array<State, numOfDataMessageIDs> states; /**< Should the corresponding message ids be replayed? */
  TypeInfo* logTypeInfo = nullptr; /**< The specifications of all the types from the log file. */
  const TypeInfo currentTypeInfo; /**< The specifications of the types in this executable. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  Thumbnail* thumbnail; /**< This will be allocated when a thumbnail was received. */
  OdometryData lastOdometryData; /** The last odometry data that was provided. Used for computing offset. */
  std::unordered_set<std::string> providedRepresentations; /** The representations that should be provided by this module. */

  // No-op update stubs
  void update(ActivationGraph&) override {}
  void update(AlternativeRobotPoseHypothesis&) override {}
  void update(AudioData&) override {}
  void update(BallModel&) override {}
  void update(BallPercept&) override {}
  void update(BallSpots&) override {}
  void update(BehaviorStatus&) override {}
  void update(BodyContour&) override {}
  void update(CameraInfo& cameraInfo) override {}
  void update(CameraMatrix& cameraMatrix) override {}
  void update(CirclePercept&) override {}
  void update(FallDownState&) override {}
  void update(FieldBoundary&) override {}
  void update(FieldColors&) override;
  void update(FieldLines&) override {}
  void update(FrameInfo&) override {}
  void update(FsrSensorData&) override {}
  void update(GameInfo&) override {}
  void update(GetUpEngineOutputLog&) override {}
  void update(GroundContactState&) override {}
  void update(GroundTruthRobotPose&) override {}
  void update(GroundTruthWorldState&) override {}
  void update(ImageCoordinateSystem& imageCoordinateSystem) override {}
  void update(InertialData&) override {}
  void update(InertialSensorData&) override {}
  void update(IntersectionsPercept&) override {}
  void update(JointAngles&) override {}
  void update(JointRequest&) override {}
  void update(JointSensorData&) override {}
  void update(KeyStates&) override {}
  void update(LabelImage&) override {}
  void update(LinesPercept&) override {}
  void update(MotionInfo&) override {}
  void update(MotionRequest&) override {}
  void update(ObstacleModel&) override {}
  void update(ObstaclesFieldPercept&) override {}
  void update(ObstaclesImagePercept&) override {}
  void update(Odometer&) override {}
  void update(OdometryData&) override {}
  void update(OpponentTeamInfo&) override {}
  void update(OwnTeamInfo&) override {}
  void update(PenaltyMarkPercept&) override {}
  void update(RawGameInfo&) override {}
  void update(RobotHealth&) override {}
  void update(RobotInfo&) override {}
  void update(RobotPose&) override {}
  void update(SelfLocalizationHypotheses&) override {}
  void update(SideConfidence&) override {}
  void update(TeamBallModel&) override {}
  void update(TeamData&) override {}
  void update(TeamPlayersModel&) override {}
  void update(WalkGeneratorData&) override {}
  void update(WalkingEngineOutput&) override {}
  void update(WalkLearner&) override {}
  void update(Whistle&) override {}

  // Updates with data
  void update(CameraImage& cameraImage) override;
  void update(ECImage& ecImage) override;
  void update(GroundTruthOdometryData&) override;

  /**
   * Handle message by writing the data contained into the corresponding blackboard entry.
   * @param message The message to be handled.
   * @return Was the message handled?
   */
  bool handle(InMessage& message);

  /**
   * The method is called for every incoming debug message by handleMessage.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  bool handleMessage2(InMessage& message);

public:
  /**
   * Default constructor.
   */
  LogDataProvider();

  /**
   * Destructor.
   */
  ~LogDataProvider();

  /**
   * The method is called for every incoming debug message.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  static bool handleMessage(InMessage& message);

  /**
   * The method returns whether idFrameFinished was received.
   * @return Were all messages of the current frame received?
   */
  static bool isFrameDataComplete();

  /**
   * Does an instance of this module exist in this thread?
   */
  static bool exists() {return theInstance != nullptr;}
};
