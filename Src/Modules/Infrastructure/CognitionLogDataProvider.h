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
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/LinePercept.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Perception/JPEGImage.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/ColorReference.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Representations/Perception/ObstacleSpots.h"
#include "Representations/Modeling/ObstacleWheel.h"
#include "Representations/Perception/BodyContour.h"

MODULE(CognitionLogDataProvider)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(CameraInfo)
  USES(CameraInfo)
  PROVIDES_WITH_OUTPUT(Image)
  REQUIRES(FieldDimensions)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
  USES(FrameInfo)
  REQUIRES(OwnTeamInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(LinePercept)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GoalPercept)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FilteredJointData)
  PROVIDES_WITH_DRAW(RobotsModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthRobotPose)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(GroundTruthBallModel)
  PROVIDES_WITH_DRAW(GroundTruthRobotsModel)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(CameraMatrix)
  REQUIRES(Image)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(ImageCoordinateSystem)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(RobotPose)
  PROVIDES_WITH_MODIFY_AND_DRAW(SideConfidence)
  PROVIDES_WITH_DRAW(ObstacleModel)
  PROVIDES(FilteredSensorData)
  PROVIDES_WITH_MODIFY(BehaviorControlOutput)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(MotionRequest)
  PROVIDES_WITH_MODIFY(HeadMotionRequest)
  PROVIDES_WITH_MODIFY(BehaviorLEDRequest)
  PROVIDES_WITH_MODIFY(ArmMotionRequest)
  PROVIDES_WITH_DRAW(CombinedWorldModel)
  PROVIDES_WITH_MODIFY(MotionInfo)
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotHealth)
  PROVIDES_WITH_MODIFY_AND_DRAW(FieldBoundary)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(ActivationGraph)
  PROVIDES_WITH_DRAW(ObstacleWheel)
  PROVIDES(ColorReference)
  PROVIDES_WITH_DRAW(ObstacleSpots)
  PROVIDES_WITH_OUTPUT_AND_DRAW(Thumbnail)
  PROVIDES_WITH_DRAW(BodyContour)
END_MODULE

class CognitionLogDataProvider : public CognitionLogDataProviderBase, public LogDataProvider
{
private:
  static PROCESS_WIDE_STORAGE(CognitionLogDataProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  BehaviorControlOutput behaviorControlOutput;

  DECLARE_DEBUG_IMAGE(corrected);

#define DISTANCE 300

  UPDATE2(Image,
  {
    DECLARE_DEBUG_DRAWING3D("representation:Image", "camera");
    IMAGE3D("representation:Image", DISTANCE, 0, 0, 0, 0, 0,
            DISTANCE * theCameraInfo.width / theCameraInfo.focalLength,
            DISTANCE * theCameraInfo.height / theCameraInfo.focalLength,
            _Image);
    DEBUG_RESPONSE("representation:JPEGImage", OUTPUT(idJPEGImage, bin, JPEGImage(_Image)););
  })
  UPDATE(CameraInfo)
  UPDATE(FrameInfo)
  UPDATE2(FieldBoundary, _FieldBoundary.width = theImage.width;);
  UPDATE(Thumbnail)
  UPDATE(ActivationGraph)
  UPDATE(ObstacleWheel)
  UPDATE(BodyContour);
  UPDATE(ObstacleSpots)
  UPDATE(RobotHealth);
  UPDATE(ColorReference)

  UPDATE(BehaviorControlOutput);
  void update(MotionRequest& motionRequest) {motionRequest = behaviorControlOutput.motionRequest;}
  void update(HeadMotionRequest& headMotionRequest) {headMotionRequest = behaviorControlOutput.headMotionRequest;}
  void update(BehaviorLEDRequest& behaviorLEDRequest) {behaviorLEDRequest = behaviorControlOutput.behaviorLEDRequest;}
  void update(ArmMotionRequest& armMotionRequest) {armMotionRequest = behaviorControlOutput.armMotionRequest;}

  UPDATE(CombinedWorldModel)
  UPDATE(ObstacleModel)
  UPDATE(FilteredSensorData)
  UPDATE(LinePercept)
  UPDATE(BallPercept)
  UPDATE(GoalPercept)
  UPDATE(BallModel)
  UPDATE(FilteredJointData)
  UPDATE(RobotsModel)
  UPDATE2(GroundTruthRobotPose, _GroundTruthRobotPose.timestamp = theFrameInfo.time;)
  UPDATE(GroundTruthBallModel)
  UPDATE(GroundTruthRobotsModel)
  UPDATE(CameraMatrix)
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
  UPDATE2(RobotPose,{_RobotPose.draw(theOwnTeamInfo.teamColor != TEAM_BLUE);})
  UPDATE(SideConfidence)
  UPDATE(MotionInfo)


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
