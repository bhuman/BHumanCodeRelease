/**
 * @file MotionLogDataProvider.cpp
 * This file implements a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "MotionLogDataProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"

MAKE_MODULE(MotionLogDataProvider, motionInfrastructure)

PROCESS_LOCAL MotionLogDataProvider* MotionLogDataProvider::theInstance = 0;

MotionLogDataProvider::MotionLogDataProvider() :
  frameDataComplete(false)
{
  theInstance = this;
}

MotionLogDataProvider::~MotionLogDataProvider()
{
  theInstance = 0;
}

void MotionLogDataProvider::update(GroundTruthOdometryData& groundTruthOdometryData)
{
  Pose2f odometryOffset(groundTruthOdometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionLogDataProvider:odometryOffsetX", odometryOffset.translation.x());
  PLOT("module:MotionLogDataProvider:odometryOffsetY", odometryOffset.translation.y());
  PLOT("module:MotionLogDataProvider:odometryOffsetRotation", odometryOffset.rotation.toDegrees());
  lastOdometryData = groundTruthOdometryData;
}

bool MotionLogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool MotionLogDataProvider::isFrameDataComplete()
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

bool MotionLogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idJointAngles:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
      {
        FrameInfo& frameInfo = (FrameInfo&) Blackboard::getInstance()["FrameInfo"];
        const JointAngles& jointAngles = (const JointAngles&) Blackboard::getInstance()["JointAngles"];
        frameInfo.cycleTime = (float)(jointAngles.timestamp - frameInfo.time) * 0.001f;
        frameInfo.time = jointAngles.timestamp;
      }
      return true;

    case idJointSensorData:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
      {
        FrameInfo& frameInfo = (FrameInfo&) Blackboard::getInstance()["FrameInfo"];
        const JointSensorData& jointSensorData = (const JointSensorData&) Blackboard::getInstance()["JointSensorData"];
        frameInfo.cycleTime = (float)(jointSensorData.timestamp - frameInfo.time) * 0.001f;
        frameInfo.time = jointSensorData.timestamp;
      }
      return true;

    case idGroundTruthOdometryData:
      if(handle(message) && Blackboard::getInstance().exists("OdometryData"))
        (OdometryData&) Blackboard::getInstance()["OdometryData"] = (OdometryData&) Blackboard::getInstance()["GroundTruthOdometryData"];
      return true;

    case idGameInfo:
      if(handle(message) && Blackboard::getInstance().exists("RawGameInfo"))
        (GameInfo&) Blackboard::getInstance()["RawGameInfo"] = (GameInfo&) Blackboard::getInstance()["GameInfo"];
      return true;

    case idProcessFinished:
      frameDataComplete = true;
      return true;

    default:
      return handle(message);
  }
}
