/**
 * @file CognitionLogDataProvider.h
 * This file declares a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "LogDataProvider.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldColors.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/DebuggingOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/ImagePatches.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/AlternativeRobotPoseHypothesis.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SelfLocalizationHypotheses.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/GoalPostPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/PlayersPercepts/PlayersFieldPercept.h"
#include "Representations/Perception/PlayersPercepts/PlayersImagePercept.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/ImageProcessing/TImage.h"

MODULE(CognitionLogDataProvider,
{,
  REQUIRES(OwnTeamInfo),
  USES(CameraInfo),
  USES(FrameInfo),
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
  PROVIDES(DebuggingOutput),
  PROVIDES(FieldBoundary),
  PROVIDES(FieldColors),
  PROVIDES(FrameInfo),
  PROVIDES(GameInfo),
  PROVIDES(GroundContactState),
  PROVIDES(GroundTruthWorldState),
  PROVIDES_WITHOUT_MODIFY(Image),
  PROVIDES(ImageCoordinateSystem),
  PROVIDES(JointSensorData),
  PROVIDES(LinesPercept),
  PROVIDES(IntersectionsPercept),
  PROVIDES(FieldLines),
  PROVIDES(GoalPostPercept),
  PROVIDES(MotionInfo),
  PROVIDES(MotionRequest),
  PROVIDES(ObstacleModel),
  PROVIDES(Odometer),
  PROVIDES(OdometryData),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RobotHealth),
  PROVIDES(RobotInfo),
  PROVIDES(PenaltyMarkPercept),
  PROVIDES(PlayersFieldPercept),
  PROVIDES(PlayersImagePercept),
  PROVIDES(RobotPose),
  PROVIDES(SelfLocalizationHypotheses),
  PROVIDES(SideConfidence),
  PROVIDES(TeamBallModel),
  PROVIDES(TeamData),
  PROVIDES(TeamPlayersModel),
  PROVIDES(Whistle),
  DEFINES_PARAMETERS(
  {,
    (bool)(true) fillImagePatchesBackground,
    (unsigned)(0x6e507850) imagePatchesBackgroundColor,
  }),
});

class CognitionLogDataProvider : public CognitionLogDataProviderBase, public LogDataProvider
{
private:
  static thread_local CognitionLogDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  LowFrameRateImage* lowFrameRateImage; /**< This will be allocated when a low frame rate image was received. */
  Image lastImages[CameraInfo::numOfCameras]; /**< Stores images per camera received as low frame rate images. */

  mutable TImage<PixelTypes::YUYVPixel> corrected;

  // No-op update stubs
  void update(ActivationGraph&) {}
  void update(AlternativeRobotPoseHypothesis&) {}
  void update(AudioData&) {}
  void update(BallModel&) {}
  void update(BallPercept&) {}
  void update(BallSpots&) {}
  void update(BehaviorStatus&) {}
  void update(BodyContour&) {}
  void update(CameraInfo& cameraInfo) {}
  void update(CameraMatrix& cameraMatrix) {}
  void update(CirclePercept&) {}
  void update(DebuggingOutput&) {}
  void update(FieldBoundary&) {}
  void update(FieldColors&);
  void update(FrameInfo&) {}
  void update(GameInfo&) {}
  void update(GroundContactState&) {}
  void update(GroundTruthWorldState&) {}
  void update(JointSensorData&) {}
  void update(FieldLines&) {}
  void update(GoalPostPercept&) {}
  void update(LinesPercept&) {}
  void update(IntersectionsPercept&) {}
  void update(MotionInfo&) {}
  void update(MotionRequest&) {}
  void update(ObstacleModel&) {}
  void update(Odometer&) {}
  void update(OdometryData&) {}
  void update(OpponentTeamInfo&) {}
  void update(OwnTeamInfo&) {}
  void update(RobotHealth&) {}
  void update(RobotInfo&) {}
  void update(PenaltyMarkPercept&) {}
  void update(PlayersFieldPercept&) {}
  void update(PlayersImagePercept&) {}
  void update(RobotPose&) {}
  void update(SelfLocalizationHypotheses&) {}
  void update(SideConfidence&) {}
  void update(TeamBallModel&) {}
  void update(TeamData&) {}
  void update(TeamPlayersModel&) {}
  void update(Whistle&) {}

  // Updates with data
  void update(Image& image);
  void update(ImageCoordinateSystem& imageCoordinateSystem);

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
  CognitionLogDataProvider();

  /**
   * Destructor.
   */
  ~CognitionLogDataProvider();

  /**
   * The method is called for every incoming debug message.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  static bool handleMessage(InMessage& message);

  /**
   * The method returns whether idProcessFinished was received.
   * @return Were all messages of the current frame received?
   */
  static bool isFrameDataComplete();
};
