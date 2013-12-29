/**
* @file CognitionLogDataProvider.cpp
* This file implements a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "CognitionLogDataProvider.h"
#include "Tools/Settings.h"
#include <vector>

PROCESS_WIDE_STORAGE(CognitionLogDataProvider) CognitionLogDataProvider::theInstance = 0;

#define ASSIGN(target, source) \
  ALLOC(target) \
  (target&) *representationBuffer[id##target] = (target&) *representationBuffer[id##source];

CognitionLogDataProvider::CognitionLogDataProvider() :
  LogDataProvider(),
  frameDataComplete(false)
{
  theInstance = this;
}

CognitionLogDataProvider::~CognitionLogDataProvider()
{
  theInstance = 0;
}

bool CognitionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool CognitionLogDataProvider::isFrameDataComplete()
{
  if(!theInstance)
    return true;
  else if(theInstance->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool CognitionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    HANDLE2(Image,
    {
      ALLOC(FrameInfo)
      FrameInfo& frameInfo = (FrameInfo&) *representationBuffer[idFrameInfo];
      const Image& image = (const Image&) *representationBuffer[idImage];
      frameInfo.cycleTime = (float) (image.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = image.timeStamp;
    })
    HANDLE(CameraInfo)
    HANDLE(FrameInfo)
    HANDLE(LinePercept)
    HANDLE(ActivationGraph)
    HANDLE(BallPercept)
    HANDLE(GoalPercept)
    HANDLE(FieldBoundary)
    HANDLE(ObstacleSpots)
    HANDLE(BallModel)
    HANDLE(ObstacleWheel)
    HANDLE(BodyContour)
    HANDLE(Thumbnail)
	HANDLE(RobotHealth)
    HANDLE2(FilteredSensorData,
    {
      ALLOC(FrameInfo)
      FrameInfo& frameInfo = (FrameInfo&) *representationBuffer[idFrameInfo];
      const FilteredSensorData& filteredSensorData = (const FilteredSensorData&) *representationBuffer[idFilteredSensorData];
      frameInfo.cycleTime = (float) (filteredSensorData.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = filteredSensorData.timeStamp;
    })
    HANDLE2(BehaviorControlOutput,
    {
      behaviorControlOutput = (const BehaviorControlOutput&) *representationBuffer[idBehaviorControlOutput];
    })
    HANDLE(ObstacleModel)
    HANDLE(FilteredJointData)
    HANDLE(RobotsModel)
    HANDLE(CombinedWorldModel)
    HANDLE2(GroundTruthRobotPose, ASSIGN(RobotPose, GroundTruthRobotPose))
    HANDLE2(GroundTruthBallModel, ASSIGN(BallModel, GroundTruthBallModel))
    HANDLE2(GroundTruthRobotsModel, ASSIGN(RobotsModel, GroundTruthRobotsModel))
    HANDLE(CameraMatrix)

    HANDLE(ImageCoordinateSystem)
    HANDLE(RobotPose)
    HANDLE(SideConfidence)
    HANDLE(MotionInfo)
    HANDLE(ColorReference)

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  case idStopwatch:
  {
    const int size = message.getMessageSize();
    std::vector<unsigned char> data;
    data.resize(size);
    message.bin.read(&data[0], size);
    Global::getDebugOut().bin.write(&data[0], size);
    Global::getDebugOut().finishMessage(idStopwatch);
    return true;
  }
  case idJPEGImage:
    ALLOC(Image)
    {
      JPEGImage jpegImage;
      message.bin >> jpegImage;
      jpegImage.toImage((Image&) *representationBuffer[idImage]);
    }
    ALLOC(FrameInfo)
    ((FrameInfo&) *representationBuffer[idFrameInfo]).time = ((Image&) *representationBuffer[idImage]).timeStamp;
    return true;

  default:
    return false;
  }
}

MAKE_MODULE(CognitionLogDataProvider, Cognition Infrastructure)
