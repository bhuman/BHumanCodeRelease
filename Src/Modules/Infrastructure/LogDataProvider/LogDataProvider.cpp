/**
 * @file LogDataProvider.cpp
 * This file implements a module that provides data replayed from a log file.
 * @author Thomas Röfer
 */

#include "LogDataProvider.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Debugging/DebugDataStreamer.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugImages.h"
#include "Debugging/Plot.h"
#include "Framework/ModuleContainer.h"
#include "Streaming/Global.h"
#include "Streaming/Streamable.h"

#include <algorithm>
#include <cstring>
#include <limits>

thread_local LogDataProvider* LogDataProvider::theInstance = nullptr;

MAKE_MODULE(LogDataProvider);

LogDataProvider::LogDataProvider() :
  frameDataComplete(false)
{
  theInstance = this;
  TypeInfo::initCurrent();
  states.fill(unknown);
  if(SystemCall::getMode() == SystemCall::logFileReplay)
    OUTPUT(idTypeInfoRequest, bin, '\0');
  ModuleContainer::addMessageHandler(handleMessage);
}

LogDataProvider::~LogDataProvider()
{
  if(logTypeInfo)
    delete logTypeInfo;

  theInstance = nullptr;
}

void LogDataProvider::update(CameraImage& cameraImage)
{
  if(SystemCall::getMode() == SystemCall::logFileReplay)
  {
    if(theCameraInfo.width / 2 != static_cast<int>(cameraImage.width) || theCameraInfo.height != static_cast<int>(cameraImage.height))
    {
      cameraImage.setResolution(theCameraInfo.width / 2, theCameraInfo.height);
      CameraImage::PixelType color;
      color.y0 = color.y1 = 0;
      color.u = color.v = 128;
      std::fill<CameraImage::PixelType*, CameraImage::PixelType>(cameraImage[0], cameraImage[cameraImage.height], color);
    }
  }

  static const float distance = 500.f;

  DECLARE_DEBUG_DRAWING3D("representation:CameraImage", "camera");
  IMAGE3D("representation:CameraImage", distance, 0, 0, 0, 0, 0,
          distance * theCameraInfo.width / theCameraInfo.focalLength,
          distance * theCameraInfo.height / theCameraInfo.focalLength,
          cameraImage);
}

void LogDataProvider::update(GroundTruthOdometryData& groundTruthOdometryData)
{
  Pose2f odometryOffset(groundTruthOdometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionLogDataProvider:odometryOffsetX", odometryOffset.translation.x());
  PLOT("module:MotionLogDataProvider:odometryOffsetY", odometryOffset.translation.y());
  PLOT("module:MotionLogDataProvider:odometryOffsetRotation", odometryOffset.rotation.toDegrees());
  lastOdometryData = groundTruthOdometryData;
}

bool LogDataProvider::handle(MessageQueue::Message message)
{
  if(message.id() == idTypeInfo)
  {
    if(!logTypeInfo)
      logTypeInfo = new TypeInfo(false);
    message.bin() >> *logTypeInfo;
    return true;
  }
  else if(SystemCall::getMode() == SystemCall::logFileReplay && !logTypeInfo)
    return false;
  else if(Blackboard::getInstance().exists(TypeRegistry::getEnumName(message.id()) + 2) && // +2 to skip the id of the messageID enums.
          ModuleGraphRunner::getInstance().getProvider(TypeRegistry::getEnumName(message.id()) + 2) == "LogDataProvider")
  {
    if(logTypeInfo)
    {
      if(states[message.id()] == unknown)
      {
        // Check whether the current and the logged specifications are the same.
        const char* type = TypeRegistry::getEnumName(message.id()) + 2;
        states[message.id()] = TypeInfo::current->areTypesEqual(*logTypeInfo, type, type) ? accept : convert;
        if(states[message.id()] == convert)
          OUTPUT_WARNING(std::string(type) + " has changed and is converted. Some fields will keep their previous values.");
      }
    }
    readMessage(message, Blackboard::getInstance()[TypeRegistry::getEnumName(message.id()) + 2]);
    return true;
  }
  else
    return false;
}

void LogDataProvider::readMessage(MessageQueue::Message message, Streamable& representation)
{
  if(states[message.id()] != convert)
    message.bin() >> representation;
  else
  {
    ASSERT(logTypeInfo);
    const char* type = TypeRegistry::getEnumName(message.id()) + 2;

    // Stream into textual representation in memory using type specification of log file.
    OutMapMemory outMap(true, 16384);
    auto stream = message.bin();
    DebugDataStreamer streamer(*logTypeInfo, stream, type);
    outMap << streamer;

    // Read from textual representation. Errors are suppressed.
    InMapMemory inMap(outMap.data(), outMap.size(), 0);
    inMap >> representation;

    // HACK: This does not work if anything else than the sample format is changed in AudioData.
    if(message.id() == idAudioData)
    {
      AudioData& audioData = dynamic_cast<AudioData&>(representation);
      for(AudioData::Sample& sample : audioData.samples)
        sample /= std::numeric_limits<short>::max();
    }
  }
}

bool LogDataProvider::handleMessage(MessageQueue::Message message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool LogDataProvider::isFrameDataComplete(bool ack)
{
  if(!theInstance)
  {
    return true;
  }
  else if(theInstance->frameDataComplete)
  {
    if(ack)
    {
      if(SystemCall::getMode() == SystemCall::logFileReplay)
        OUTPUT(idLogResponse, bin, '\0');
      theInstance->frameDataComplete = false;
    }
    return true;
  }
  else
    return false;
}

bool LogDataProvider::handleMessage2(MessageQueue::Message message)
{
  switch(message.id())
  {
    case idCameraImage:
      handle<CameraImage, FrameInfo>(message, "CameraImage", "FrameInfo",
                                     [&](CameraImage& source, FrameInfo& target)
                                     {target.time = source.timestamp;});
      return true;

    case idCameraInfo:
      handle<CameraInfo, ImageCoordinateSystem>(message, "CameraInfo", "ImageCoordinateSystem",
                                                [&](CameraInfo& source, ImageCoordinateSystem& target)
                                                {target.cameraInfo = source;});
      return true;

    case idImageCoordinateSystem:
      handle<ImageCoordinateSystem, CameraInfo>(message, "ImageCoordinateSystem", "CameraInfo",
                                                [&](ImageCoordinateSystem& target, CameraInfo& source)
                                                {target.cameraInfo = source;});
      return true;

    case idFrameInfo:
      handle<FrameInfo, CameraImage>(message, "FrameInfo", "CameraImage",
                                     [&](FrameInfo& source, CameraImage& target)
                                     {target.timestamp = source.time;});
      return true;

    case idGroundTruthOdometryData:
      handle<GroundTruthOdometryData, OdometryData>(message, "GroundTruthOdometryData", "OdometryData",
                                                    [&](GroundTruthOdometryData& source, OdometryData& target)
                                                    {target = source;});
      return true;

    case idJointAngles:
      handle<JointAngles, FrameInfo>(message, "JointAngles", "FrameInfo",
                                     [&](JointAngles& source, FrameInfo& target)
                                     {target.time = source.timestamp;});
      return true;

    case idJointSensorData:
      handle<JointSensorData, FrameInfo>(message, "JointSensorData", "FrameInfo",
                                         [&](JointSensorData& source, FrameInfo& target)
                                         {target.time = source.timestamp;});
      return true;

    case idRobotPose:
      if(ModuleGraphRunner::getInstance().getProvider("GroundTruthRobotPose") == "LogDataProvider")
      {
        handle<RobotPose, RobotPose>(message, "RobotPose", "GroundTruthRobotPose",
                                     [&](RobotPose& source, RobotPose& target)
                                     {target = source;});
        if(Blackboard::getInstance().exists("FrameInfo"))
          static_cast<GroundTruthRobotPose&>(Blackboard::getInstance()["GroundTruthRobotPose"]).timestamp = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]).time;
      }
      else
        handle(message);
      return true;

    case idJPEGImage:
      if(ModuleGraphRunner::getInstance().getProvider("CameraImage") == "LogDataProvider")
      {
        JPEGImage jpegImage;
        message.bin() >> jpegImage;
        jpegImage.toCameraImage(static_cast<CameraImage&>(Blackboard::getInstance()["CameraImage"]));
        if(Blackboard::getInstance().exists("FrameInfo"))
          static_cast<FrameInfo&>(Blackboard::getInstance()["FrameInfo"]).time = static_cast<const CameraImage&>(Blackboard::getInstance()["CameraImage"]).timestamp;
      }
      return true;

    case idFrameFinished:
      frameDataComplete = true;
      return true;

    case idStopwatch:
    {
      DEBUG_RESPONSE_NOT("timing")
        Global::getDebugOut() << message;
      return true;
    }

    case idAnnotation:
    {
      Global::getDebugOut() << message;
      return true;
    }

    default:
      return handle(message);
  }
}
