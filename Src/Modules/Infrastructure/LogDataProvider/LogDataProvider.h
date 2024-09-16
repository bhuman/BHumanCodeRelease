/**
 * @file LogDataProvider.h
 * This file declares a module that provides data replayed from a log file.
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/AgentStates.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/InitialToReady.h"
#include "Representations/BehaviorControl/SharedAutonomyRequest.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/GameControllerData.h"
#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/AlternativeRobotPoseHypothesis.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SelfLocalizationHypotheses.h"
#include "Representations/Modeling/SideInformation.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/GoalPercepts/GoalPostsPercept.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/RefereePercept/Keypoints.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/InertialSensorData.h"
#include "Representations/Sensing/JointAnglePred.h"
#include "Representations/Sensing/JointPlay.h"
#include "Representations/Sensing/RobotStableState.h"
#include "ImageProcessing/Image.h"
#include "ImageProcessing/PixelTypes.h"
#include "Streaming/MessageIDs.h"
#include "Framework/Module.h"
#include "Framework/ModuleGraphRunner.h"
#include "Streaming/TypeInfo.h"
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
  PROVIDES(AgentStates),
  PROVIDES(AlternativeRobotPoseHypothesis),
  PROVIDES(ArmMotionRequest),
  PROVIDES(AudioData),
  PROVIDES(BallModel),
  PROVIDES(BallPercept),
  PROVIDES(BallSpots),
  PROVIDES(BehaviorStatus),
  PROVIDES(BodyContour),
  PROVIDES(CameraCalibration),
  PROVIDES(CameraInfo),
  PROVIDES(CameraMatrix),
  PROVIDES(CirclePercept),
  PROVIDES(ECImage),
  PROVIDES(FallDownState),
  PROVIDES(FieldBoundary),
  PROVIDES(FieldLines),
  PROVIDES(FootOffset),
  PROVIDES(FootSupport),
  PROVIDES(FrameInfo),
  PROVIDES(FsrData),
  PROVIDES(FsrSensorData),
  PROVIDES(GameControllerData),
  PROVIDES(GameState),
  PROVIDES(GlobalOpponentsModel),
  PROVIDES(GlobalTeammatesModel),
  PROVIDES(GoalPostsPercept),
  PROVIDES(GyroOffset),
  PROVIDES(GroundContactState),
  PROVIDES(GroundTruthOdometryData),
  PROVIDES(GroundTruthRobotPose),
  PROVIDES(GroundTruthWorldState),
  PROVIDES(HeadMotionRequest),
  PROVIDES(HeadMotionInfo),
  PROVIDES(ImageCoordinateSystem),
  PROVIDES(IMUCalibration),
  PROVIDES(IMUValueState),
  PROVIDES(IndirectKick),
  PROVIDES(InertialData),
  PROVIDES(InertialSensorData),
  PROVIDES(InitialToReady),
  PROVIDES(IntersectionsPercept),
  PROVIDES(JointAnglePred),
  PROVIDES(JointAngles),
  PROVIDES(JointCalibration),
  PROVIDES(JointPlay),
  PROVIDES(JointRequest),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(Keypoints),
  PROVIDES(LinesPercept),
  PROVIDES(MotionInfo),
  PROVIDES(MotionRequest),
  PROVIDES(MotionRobotHealth),
  PROVIDES(ObstacleModel),
  PROVIDES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  PROVIDES(Odometer),
  PROVIDES(OdometryData),
  PROVIDES(OdometryDataPreview),
  PROVIDES(OdometryTranslationRequest),
  PROVIDES(PenaltyMarkPercept),
  PROVIDES(RawInertialSensorData),
  PROVIDES(ReceivedTeamMessages),
  PROVIDES(RefereePercept),
  PROVIDES(RobotHealth),
  PROVIDES(RobotPose),
  PROVIDES(RobotStableState),
  PROVIDES(SelfLocalizationHypotheses),
  PROVIDES(SharedAutonomyRequest),
  PROVIDES(SideInformation),
  PROVIDES(SkillRequest),
  PROVIDES(StrategyStatus),
  PROVIDES(TeammatesBallModel),
  PROVIDES(TeamData),
  PROVIDES(WalkStepData),
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
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  OdometryData lastOdometryData; /**< The last odometry data that was provided. Used for computing offset. */

  // No-op update stubs
  void update(ActivationGraph&) override {}
  void update(AgentStates&) override {}
  void update(AlternativeRobotPoseHypothesis&) override {}
  void update(ArmMotionRequest&) override {}
  void update(AudioData&) override {}
  void update(BallModel&) override {}
  void update(BallPercept&) override {}
  void update(BallSpots&) override {}
  void update(BehaviorStatus&) override {}
  void update(BodyContour&) override {}
  void update(CameraCalibration&) override {}
  void update(CameraInfo&) override {}
  void update(CameraMatrix&) override {}
  void update(CirclePercept&) override {}
  void update(ECImage&) override {}
  void update(FallDownState&) override {}
  void update(FieldBoundary&) override {}
  void update(FieldLines&) override {}
  void update(FootOffset&) override {}
  void update(FootSupport&) override {}
  void update(FrameInfo&) override {}
  void update(FsrData&) override {}
  void update(FsrSensorData&) override {}
  void update(GameControllerData&) override {}
  void update(GameState&) override {}
  void update(GlobalOpponentsModel&) override {}
  void update(GlobalTeammatesModel&) override {}
  void update(GoalPostsPercept&) override {}
  void update(GyroOffset&) override {}
  void update(IMUValueState&) override {}
  void update(GroundContactState&) override {}
  void update(GroundTruthRobotPose&) override {}
  void update(GroundTruthWorldState&) override {}
  void update(HeadMotionInfo&) override {}
  void update(HeadMotionRequest&) override {}
  void update(ImageCoordinateSystem&) override {}
  void update(IMUCalibration&) override {}
  void update(InertialData&) override {}
  void update(IndirectKick&) override {}
  void update(InertialSensorData&) override {}
  void update(InitialToReady&) override {}
  void update(IntersectionsPercept&) override {}
  void update(JointAnglePred&) override {}
  void update(JointAngles&) override {}
  void update(JointCalibration&) override {}
  void update(JointPlay&) override {}
  void update(JointRequest&) override {}
  void update(JointSensorData&) override {}
  void update(KeyStates&) override {}
  void update(Keypoints&) override {}
  void update(LinesPercept&) override {}
  void update(MotionInfo&) override {}
  void update(MotionRequest&) override {}
  void update(MotionRobotHealth&) override {}
  void update(ObstacleModel&) override {}
  void update(ObstaclesFieldPercept&) override {}
  void update(ObstaclesImagePercept&) override {}
  void update(Odometer&) override {}
  void update(OdometryData&) override {}
  void update(OdometryDataPreview&) override {}
  void update(OdometryTranslationRequest&) override {}
  void update(PenaltyMarkPercept&) override {}

  void update(RawInertialSensorData&) override
  {}
  void update(ReceivedTeamMessages&) override {}
  void update(RefereePercept&) override {}
  void update(RobotHealth&) override {}
  void update(RobotPose&) override {}
  void update(RobotStableState&) override {}
  void update(SelfLocalizationHypotheses&) override {}
  void update(SharedAutonomyRequest&) override {}
  void update(SideInformation&) override {}
  void update(SkillRequest&) override {}
  void update(StrategyStatus&) override {}
  void update(TeammatesBallModel&) override {}
  void update(TeamData&) override {}
  void update(WalkStepData&) override {}
  void update(WalkingEngineOutput&) override {}
  void update(WalkLearner&) override {}
  void update(Whistle&) override {}

  // Updates with data
  void update(CameraImage& cameraImage) override;
  void update(GroundTruthOdometryData&) override;

  /**
   * Handle message by writing the data contained into the corresponding blackboard entry.
   * @param message The message to be handled.
   * @return Was the message handled?
   */
  bool handle(MessageQueue::Message message);

  /**
   * Read a representation from a message.
   * @param message The message to read from.
   * @param representation The representation the message is stored in.
   */
  void readMessage(MessageQueue::Message message, Streamable& representation);

  /**
   * The method is called for every incoming debug message by handleMessage.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  bool handleMessage2(MessageQueue::Message message);

  /**
   * Handle a message that should update a representation, but also involves
   * a second representation. The second representation is either updated as
   * well or it contains data that is copied to the representation that is handled.
   * At least one of the two representations must be provided by this module,
   * otherwise nothing happens.
   * @param message The message containing the representation handled.
   * @param read The name of the representation handled.
   * @param additional The name of the additional representation.
   * @param update A function that received both representations, the
   *        handled one first and the additional one second.
   */
  template<typename Read, typename Additional> void handle(MessageQueue::Message message, const char* read, const char* additional,
                                                           const std::function<void(Read&, Additional&)>& update)
  {
    const bool provided = handle(message);
    if((provided && Blackboard::getInstance().exists(additional)) ||
       ModuleGraphRunner::getInstance().getProvider(additional) == "LogDataProvider")
    {
      if(provided)
        update(static_cast<Read&>(Blackboard::getInstance()[read]), static_cast<Additional&>(Blackboard::getInstance()[additional]));
      else
      {
        Read representation;
        readMessage(message, representation);
        update(representation, static_cast<Additional&>(Blackboard::getInstance()[additional]));
      }
    }
  }

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
  static bool handleMessage(MessageQueue::Message message);

  /**
   * The method returns whether idFrameFinished was received.
   * @param ack Acknowledge to the log player that the frame was received.
   * @return Were all messages of the current frame received?
   */
  static bool isFrameDataComplete(bool ack = true);

  /**
   * Does an instance of this module exist in this thread?
   */
  static bool exists() {return theInstance != nullptr;}
};
