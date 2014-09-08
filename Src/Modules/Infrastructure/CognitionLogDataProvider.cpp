/**
* @file CognitionLogDataProvider.cpp
* This file implements a module that provides data replayed from a log file.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "CognitionLogDataProvider.h"
#include "Representations/Infrastructure/Thumbnail.h"
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
    HANDLE(OwnTeamInfo)
    HANDLE(OpponentTeamInfo)
    HANDLE(GameInfo)
    HANDLE2(RobotInfo, ((RobotInfo&) *representationBuffer[idRobotInfo]).number = Global::getSettings().playerNumber;)
    HANDLE(AudioData)
    HANDLE2(Image,
    {
      ALLOC(FrameInfo)
      FrameInfo& frameInfo = (FrameInfo&) *representationBuffer[idFrameInfo];
      const Image& image = (const Image&) *representationBuffer[idImage];
      frameInfo.cycleTime = (float) (image.timeStamp - frameInfo.time) * 0.001f;
      frameInfo.time = image.timeStamp;
    })
    HANDLE(LowFrameRateImage)
    HANDLE(CameraInfo)
    HANDLE2(FrameInfo,
    {
      ALLOC(Image)
      Image& image = (Image&) *representationBuffer[idImage];
      const FrameInfo& frameInfo = (FrameInfo&) *representationBuffer[idFrameInfo];
      image.timeStamp = frameInfo.time;
    })
    HANDLE(LinePercept)
    HANDLE(ActivationGraph)
    HANDLE(BallPercept)
    HANDLE(GoalPercept)
    HANDLE(FieldBoundary)
    HANDLE(BallModel)
    HANDLE(ObstacleWheel)
    HANDLE(BodyContour)
    HANDLE2(Thumbnail,
    {
      ALLOC(Image)
      Thumbnail& thumbnail = (Thumbnail&) *representationBuffer[idThumbnail];
      thumbnail.toImage((Image&) *representationBuffer[idImage]);
      return true;
    })
    HANDLE(TeammateReliability)
    HANDLE2(TeammateDataCompressed,
    {
      ALLOC(TeammateData)
      TeammateData& teammateData = (TeammateData&) *representationBuffer[idTeammateData];
      TeammateDataCompressed& teammateDataCompressed = (TeammateDataCompressed&) *representationBuffer[idTeammateDataCompressed];
      teammateData = TeammateData(teammateDataCompressed);
    })
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
    HANDLE(CombinedWorldModel)
    HANDLE(CameraMatrix)
    HANDLE(RobotPercept)
    HANDLE(ImageCoordinateSystem)
    HANDLE(LineSpots)
    HANDLE(LocalizationTeamBall)
    HANDLE(RobotPose)
    HANDLE(SideConfidence)
    HANDLE(MotionInfo)
    HANDLE(GroundTruthWorldState)
    HANDLE(Odometer)
    HANDLE(GroundContactState)
    HANDLE(ReceivedSPLStandardMessages)

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
