/**
 * @file CognitionLogDataProvider.h
 * This file declares a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "LogDataProvider.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/ReceivedSPLStandardMessages.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/LocalizationTeamBall.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Perception/PlayersPercept.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Module/Module.h"

MODULE(CognitionLogDataProvider,
{,
  REQUIRES(OwnTeamInfo),
  USES(CameraInfo),
  USES(FrameInfo),
  PROVIDES(ActivationGraph),
  PROVIDES(AudioData),
  PROVIDES(BallModel),
  PROVIDES(BallPercept),
  PROVIDES(BehaviorStatus),
  PROVIDES(BodyContour),
  PROVIDES(CameraInfo),
  PROVIDES(CameraMatrix),
  PROVIDES(FieldBoundary),
  PROVIDES(FrameInfo),
  PROVIDES(GameInfo),
  PROVIDES(GoalPercept),
  PROVIDES(GroundContactState),
  PROVIDES(GroundTruthWorldState),
  PROVIDES_WITHOUT_MODIFY(Image),
  PROVIDES(ImageCoordinateSystem),
  PROVIDES(JointSensorData),
  PROVIDES(LineSpots),
  PROVIDES(LinePercept),
  PROVIDES(LocalizationTeamBall),
  PROVIDES(MotionInfo),
  PROVIDES(MotionRequest),
  PROVIDES(ObstacleModel),
  PROVIDES(Odometer),
  PROVIDES(OdometryData),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(ReceivedSPLStandardMessages),
  PROVIDES(RobotHealth),
  PROVIDES(RobotInfo),
  PROVIDES(PlayersPercept),
  PROVIDES(RobotPose),
  PROVIDES(ScanlineRegions),
  PROVIDES(SideConfidence),
  PROVIDES(TeamBallModel),
  PROVIDES(TeammateData),
  PROVIDES(TeammateReliability),
  PROVIDES(TeamPlayersModel),
});

class CognitionLogDataProvider : public CognitionLogDataProviderBase, public LogDataProvider
{
private:
  static PROCESS_LOCAL CognitionLogDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  LowFrameRateImage* lowFrameRateImage; /**< This will be allocated when a low frame rate image was received. */
  Image lastImages[CameraInfo::numOfCameras]; /**< Stores images per camera received as low frame rate images. */

  DECLARE_DEBUG_IMAGE(corrected);

  void update(ActivationGraph&) {}
  void update(AudioData&) {}
  void update(BallModel&) {}
  void update(BallPercept&) {}
  void update(BehaviorStatus&) {}
  void update(BodyContour&) {}
  void update(CameraInfo& cameraInfo) {}
  void update(CameraMatrix&) {}
  void update(FieldBoundary&) {}
  void update(FrameInfo&) {}
  void update(GameInfo&) {}
  void update(GoalPercept&) {}
  void update(GroundContactState&) {}
  void update(GroundTruthWorldState&) {}
  void update(Image& image);
  void update(ImageCoordinateSystem& imageCoordinateSystem);
  void update(JointSensorData&) {}
  void update(LinePercept&) {}
  void update(LineSpots&) {}
  void update(LocalizationTeamBall&) {}
  void update(MotionInfo&) {}
  void update(MotionRequest&) {}
  void update(ObstacleModel&) {}
  void update(Odometer&) {}
  void update(OdometryData&) {}
  void update(OpponentTeamInfo&) {}
  void update(OwnTeamInfo&) {}
  void update(ReceivedSPLStandardMessages&) {}
  void update(RobotHealth&) {}
  void update(RobotInfo&) {}
  void update(PlayersPercept&) {}
  void update(RobotPose&) {}
  void update(SideConfidence&) {}
  void update(ScanlineRegions&) {}
  void update(TeamBallModel&) {}
  void update(TeammateData&) {}
  void update(TeammateReliability&) {}
  void update(TeamPlayersModel&) {}

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
