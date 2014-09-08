/**
* @file CognitionLogDataProvider.h
* This file declares a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once
//TODO clean up includes
#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "LogDataProvider.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/ReceivedSPLStandardMessages.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Modeling/ObstacleWheel.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Modeling/LocalizationTeamBall.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Sensing/GroundContactState.h"

MODULE(CognitionLogDataProvider,
{,
  REQUIRES(OwnTeamInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(OwnTeamInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(OpponentTeamInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GameInfo),
  PROVIDES_WITH_MODIFY(RobotInfo),
  PROVIDES_WITH_OUTPUT(AudioData),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(CameraInfo),
  USES(CameraInfo),
  PROVIDES(CameraInfoFullRes),
  PROVIDES_WITH_OUTPUT(Image),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo),
  USES(FrameInfo),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePercept),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GoalPercept),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModel),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FilteredJointData),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrix),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(ImageCoordinateSystem),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(RobotPose),
  PROVIDES_WITH_MODIFY_AND_DRAW(SideConfidence),
  PROVIDES_WITH_DRAW(ObstacleModel),
  PROVIDES(FilteredSensorData),
  PROVIDES_WITH_MODIFY(BehaviorControlOutput),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest),
  PROVIDES_WITH_MODIFY(HeadMotionRequest),
  PROVIDES_WITH_MODIFY(BehaviorLEDRequest),
  PROVIDES_WITH_MODIFY(ArmMotionRequest),
  PROVIDES_WITH_DRAW(CombinedWorldModel),
  PROVIDES_WITH_MODIFY(MotionInfo),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotHealth),
  PROVIDES_WITH_MODIFY_AND_DRAW(FieldBoundary),
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ActivationGraph),
  PROVIDES_WITH_DRAW(ObstacleWheel),
  PROVIDES_WITH_DRAW(BodyContour),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotPercept),
  PROVIDES_WITH_MODIFY(GroundTruthWorldState),
  PROVIDES_WITH_DRAW(LineSpots),
  PROVIDES_WITH_MODIFY(Odometer),
  PROVIDES_WITH_MODIFY(GroundContactState),
  PROVIDES_WITH_MODIFY_AND_DRAW(LocalizationTeamBall),
  PROVIDES(TeammateDataCompressed),
  PROVIDES_WITH_DRAW(TeammateData),
  PROVIDES_WITH_DRAW(TeammateReliability),
  PROVIDES_WITH_MODIFY(ReceivedSPLStandardMessages),
});

class CognitionLogDataProvider : public CognitionLogDataProviderBase, public LogDataProvider
{
private:
  static PROCESS_WIDE_STORAGE(CognitionLogDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  BehaviorControlOutput behaviorControlOutput;
  Image lastImage[CameraInfo::numOfCameras];

  DECLARE_DEBUG_IMAGE(corrected);

#define DISTANCE 300

  void update(Image& image)
  {
    if(representationBuffer[idLowFrameRateImage])
    {
      CameraInfo& info = (CameraInfo&) *representationBuffer[idCameraInfo];
      LowFrameRateImage& lfri = (LowFrameRateImage&) *representationBuffer[idLowFrameRateImage];
      if(lfri.imageUpdated)
      {
        lastImage[info.camera] = lfri.image;
        image = lfri.image;
      }
      else
        image = lastImage[info.camera];
    }
    else if(representationBuffer[idImage])
      image = *((Image*)representationBuffer[idImage]);

    DECLARE_DEBUG_DRAWING3D("representation:Image", "camera");
    IMAGE3D("representation:Image", DISTANCE, 0, 0, 0, 0, 0,
            DISTANCE * theCameraInfo.width / theCameraInfo.focalLength,
            DISTANCE * theCameraInfo.height / theCameraInfo.focalLength,
            image);
    DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(image)););
  }
  UPDATE(RobotPercept)
  UPDATE(OwnTeamInfo)
  UPDATE(OpponentTeamInfo)
  UPDATE(GameInfo)
  UPDATE(RobotInfo)
  UPDATE(AudioData)
  UPDATE(CameraInfo)
  void update(CameraInfoFullRes& cameraInfoFullRes)
  {
    if(representationBuffer[idCameraInfo])
      cameraInfoFullRes = *((CameraInfo*) representationBuffer[idCameraInfo]);
  }
  UPDATE(FrameInfo)
  UPDATE(FieldBoundary);
  UPDATE(ActivationGraph)
  UPDATE(ObstacleWheel)
  UPDATE(BodyContour);
  UPDATE(RobotHealth);

  UPDATE(BehaviorControlOutput);
  void update(MotionRequest& motionRequest) {motionRequest = behaviorControlOutput.motionRequest;}
  void update(HeadMotionRequest& headMotionRequest) {headMotionRequest = behaviorControlOutput.headMotionRequest;}
  void update(BehaviorLEDRequest& behaviorLEDRequest) {behaviorLEDRequest = behaviorControlOutput.behaviorLEDRequest;}
  void update(ArmMotionRequest& armMotionRequest) {armMotionRequest = behaviorControlOutput.armMotionRequest;}

  UPDATE(CombinedWorldModel)
  UPDATE(ObstacleModel)
  UPDATE(FilteredSensorData)
  UPDATE(LinePercept)
  UPDATE(LineSpots)
  UPDATE(BallPercept)
  UPDATE(GoalPercept)
  UPDATE(BallModel)
  UPDATE(LocalizationTeamBall)
  UPDATE(FilteredJointData)
  UPDATE(CameraMatrix)
  UPDATE(TeammateDataCompressed)
  UPDATE(TeammateData)
  UPDATE(TeammateReliability)
  UPDATE2(ImageCoordinateSystem,
  {
    _ImageCoordinateSystem.setCameraInfo(theCameraInfo);
    DECLARE_DEBUG_DRAWING("loggedHorizon", "drawingOnImage"); // displays the horizon
    ARROW("loggedHorizon",
          _ImageCoordinateSystem.origin.x,
          _ImageCoordinateSystem.origin.y,
          _ImageCoordinateSystem.origin.x + _ImageCoordinateSystem.rotation.c[0].x * 50,
          _ImageCoordinateSystem.origin.y + _ImageCoordinateSystem.rotation.c[0].y * 50,
          0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    ARROW("loggedHorizon",
          _ImageCoordinateSystem.origin.x,
          _ImageCoordinateSystem.origin.y,
          _ImageCoordinateSystem.origin.x + _ImageCoordinateSystem.rotation.c[1].x * 50,
          _ImageCoordinateSystem.origin.y + _ImageCoordinateSystem.rotation.c[1].y * 50,
          0, Drawings::ps_solid, ColorRGBA(255, 0, 0));
    COMPLEX_DEBUG_IMAGE(corrected,
    {
      Image* i = (Image*) representationBuffer[idImage];
      if(i)
      {
        INIT_DEBUG_IMAGE_BLACK(corrected, theCameraInfo.width, theCameraInfo.height);
        int yDest = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y;
        for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
          for(int yDest2 = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y; yDest <= yDest2; ++yDest)
          {
            int xDest = -_ImageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x;
            for(int xSrc = 0; xSrc < theCameraInfo.width; ++xSrc)
            {
              for(int xDest2 = -_ImageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x; xDest <= xDest2; ++xDest)
              {
                DEBUG_IMAGE_SET_PIXEL_YUV(corrected, xDest + int(theCameraInfo.opticalCenter.x + 0.5f),
                yDest + int(theCameraInfo.opticalCenter.y + 0.5f),
                (*i)[ySrc][xSrc].y,
                (*i)[ySrc][xSrc].cb,
                (*i)[ySrc][xSrc].cr);
              }
            }
          }
        SEND_DEBUG_IMAGE(corrected);
      }
    });
  })
  UPDATE(RobotPose)
  UPDATE(SideConfidence)
  UPDATE(MotionInfo)
  UPDATE(GroundTruthWorldState)
  UPDATE(Odometer)
  UPDATE(GroundContactState)
  UPDATE(ReceivedSPLStandardMessages)

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
