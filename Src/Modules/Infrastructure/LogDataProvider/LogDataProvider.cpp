/**
 * @file LogDataProvider.cpp
 * This file implements a module that provides data replayed from a log file.
 * @author Thomas Röfer
 */

#include "LogDataProvider.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/Thumbnail.h"
#include "Tools/Debugging/DebugDataStreamer.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Framework/ModuleContainer.h"
#include "Tools/Global.h"
#include "Tools/Streams/Streamable.h"

#include <algorithm>
#include <cstring>
#include <limits>

thread_local LogDataProvider* LogDataProvider::theInstance = nullptr;

MAKE_MODULE(LogDataProvider, infrastructure)

LogDataProvider::LogDataProvider() :
  frameDataComplete(false),
  thumbnail(nullptr)
{
  theInstance = this;
  states.fill(unknown);
  if(SystemCall::getMode() == SystemCall::logFileReplay)
    OUTPUT(idTypeInfoRequest, bin, '\0');
  ModuleContainer::addMessageHandler(handleMessage);
}

LogDataProvider::~LogDataProvider()
{
  if(thumbnail)
    delete thumbnail;
  if(logTypeInfo)
    delete logTypeInfo;

  theInstance = nullptr;
}

void LogDataProvider::update(CameraImage& cameraImage)
{
  if(SystemCall::getMode() == SystemCall::logFileReplay)
  {
    if(thumbnail)
      thumbnail->toCameraImage(cameraImage);
    else if(theCameraInfo.width / 2 != static_cast<int>(cameraImage.width) || theCameraInfo.height != static_cast<int>(cameraImage.height))
    {
      cameraImage.setResolution(theCameraInfo.width / 2, theCameraInfo.height);
      CameraImage::PixelType color;
      color.y0 = color.y1 = 0;
      color.u = color.v = 128;
      std::fill<CameraImage::PixelType*, CameraImage::PixelType>(cameraImage[0], cameraImage[cameraImage.height], color);
    }
  }

  static const float distance = 300.f;

  DECLARE_DEBUG_DRAWING3D("representation:CameraImage", "camera");
  IMAGE3D("representation:CameraImage", distance, 0, 0, 0, 0, 0,
          distance * theCameraInfo.width / theCameraInfo.focalLength,
          distance * theCameraInfo.height / theCameraInfo.focalLength,
          cameraImage);
  DEBUG_RESPONSE("representation:JPEGImage") OUTPUT(idJPEGImage, bin, JPEGImage(cameraImage));
}

void LogDataProvider::update(ECImage& ecImage)
{
  if(SystemCall::getMode() == SystemCall::logFileReplay && thumbnail)
    thumbnail->toECImage(ecImage);
}

void LogDataProvider::update(FieldColors& fieldColors)
{
  DEBUG_RESPONSE_ONCE("representation:FieldColors:once")
    OUTPUT(idFieldColors, bin, fieldColors);
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

bool LogDataProvider::handle(InMessage& message)
{
  if(message.getMessageID() == idTypeInfo)
  {
    if(!logTypeInfo)
      logTypeInfo = new TypeInfo(false);
    message.bin >> *logTypeInfo;
    return true;
  }
  else if(message.getMessageID() == idModuleRequest)
  {
    ModuleGraphCreator::ExecutionValues values;
    message.bin >> values;
    providedRepresentations.clear();
    for(const Configuration::RepresentationProvider& representationProvider : values.providers)
      if(representationProvider.provider == "LogDataProvider")
        providedRepresentations.insert(representationProvider.representation);
    return true;
  }
  else if(SystemCall::getMode() == SystemCall::logFileReplay && !logTypeInfo)
    return false;
  else if(Blackboard::getInstance().exists(TypeRegistry::getEnumName(message.getMessageID()) + 2)) // +2 to skip the id of the messageID enums.
  {
    if(logTypeInfo)
    {
      if(states[message.getMessageID()] == unknown)
      {
        // Check whether the current and the logged specifications are the same.
        const char* type = TypeRegistry::getEnumName(message.getMessageID()) + 2;
        states[message.getMessageID()] = currentTypeInfo.areTypesEqual(*logTypeInfo, type, type) ? accept : convert;
        if(states[message.getMessageID()] == convert)
          OUTPUT_WARNING(std::string(type) + " has changed and is converted. Some fields will keep their previous values.");
      }
    }
    if(states[message.getMessageID()] != convert)
      message.bin >> Blackboard::getInstance()[TypeRegistry::getEnumName(message.getMessageID()) + 2];
    else
    {
      ASSERT(logTypeInfo);
      const char* type = TypeRegistry::getEnumName(message.getMessageID()) + 2;

      // Stream into textual representation in memory using type specification of log file.
      OutMapMemory outMap(true, 16384);
      DebugDataStreamer streamer(*logTypeInfo, message.bin, type);
      outMap << streamer;

      // Read from textual representation. Errors are suppressed.
      InMapMemory inMap(outMap.data(), outMap.size(), false);
      inMap >> Blackboard::getInstance()[TypeRegistry::getEnumName(message.getMessageID()) + 2];

      // HACK: This does not work if anything else than the sample format is changed in AudioData.
      if(message.getMessageID() == idAudioData)
      {
        AudioData& audioData = dynamic_cast<AudioData&>(Blackboard::getInstance()[TypeRegistry::getEnumName(message.getMessageID()) + 2]);
        for(AudioData::Sample& sample : audioData.samples)
          sample /= std::numeric_limits<short>::max();
      }
    }
    return true;
  }
  else
    return false;
}

bool LogDataProvider::handleMessage(InMessage& message)
{
  return theInstance && theInstance->handleMessage2(message);
}

bool LogDataProvider::isFrameDataComplete()
{
  if(!theInstance)
  {
    return true;
  }
  else if(theInstance->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    theInstance->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool LogDataProvider::handleMessage2(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idCameraImage:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
        static_cast<FrameInfo&>(Blackboard::getInstance()["FrameInfo"]).time = static_cast<const CameraImage&>(Blackboard::getInstance()["CameraImage"]).timestamp;
      return true;

    case idCameraInfo:
      if(handle(message) && Blackboard::getInstance().exists("ImageCoordinateSystem"))
        static_cast<ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]).cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      return true;

    case idImageCoordinateSystem:
      if(handle(message) && Blackboard::getInstance().exists("CameraInfo"))
        static_cast<ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]).cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      return true;

    case idFrameInfo:
      if(handle(message) && Blackboard::getInstance().exists("CameraImage"))
        static_cast<CameraImage&>(Blackboard::getInstance()["CameraImage"]).timestamp = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]).time;
      return true;

    case idGameInfo:
      if(handle(message) && Blackboard::getInstance().exists("RawGameInfo"))
        static_cast<GameInfo&>(Blackboard::getInstance()["RawGameInfo"]) = static_cast<GameInfo&>(Blackboard::getInstance()["GameInfo"]);
      return true;

    case idGroundTruthOdometryData:
      if(handle(message) && Blackboard::getInstance().exists("OdometryData"))
        static_cast<OdometryData&>(Blackboard::getInstance()["OdometryData"]) = static_cast<OdometryData&>(Blackboard::getInstance()["GroundTruthOdometryData"]);
      return true;

    case idJointAngles:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
      {
        FrameInfo& frameInfo = static_cast<FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
        const JointAngles& jointAngles = static_cast<const JointAngles&>(Blackboard::getInstance()["JointAngles"]);
        if(jointAngles.timestamp != frameInfo.time) // Do not change frameInfo, when it is provided by a log file.
          frameInfo.time = jointAngles.timestamp;
      }
      return true;

    case idJointSensorData:
      if(handle(message) && Blackboard::getInstance().exists("FrameInfo"))
      {
        FrameInfo& frameInfo = static_cast<FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
        const JointSensorData& jointSensorData = static_cast<const JointSensorData&>(Blackboard::getInstance()["JointSensorData"]);
        if(jointSensorData.timestamp != frameInfo.time) // Do not change frameInfo, when it is provided by a log file.
          frameInfo.time = jointSensorData.timestamp;
      }
      return true;

    case idRobotPose:
      if(handle(message) && Blackboard::getInstance().exists("GroundTruthRobotPose") && providedRepresentations.count("GroundTruthRobotPose"))
      {
        static_cast<RobotPose&>(Blackboard::getInstance()["GroundTruthRobotPose"]) = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
        if(Blackboard::getInstance().exists("FrameInfo"))
          static_cast<GroundTruthRobotPose&>(Blackboard::getInstance()["GroundTruthRobotPose"]).timestamp = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]).time;
      }
      return true;

    case idThumbnail:
      if(Blackboard::getInstance().exists("CameraImage") || Blackboard::getInstance().exists("ECImage"))
      {
        if(!thumbnail)
          thumbnail = new Thumbnail;
        message.bin >> *thumbnail;
      }
      return true;

    case idFrameFinished:
      frameDataComplete = true;
      return true;

    case idStopwatch:
    {
      DEBUG_RESPONSE_NOT("timing")
      {
        const int size = message.getMessageSize();
        std::vector<unsigned char> data;
        data.resize(size);
        message.bin.read(&data[0], size);
        Global::getDebugOut().bin.write(&data[0], size);
        Global::getDebugOut().finishMessage(idStopwatch);
      }
      return true;
    }

    case idAnnotation:
    {
      const int size = message.getMessageSize();
      std::vector<unsigned char> data;
      data.resize(size);
      message.bin.read(&data[0], size);
      Global::getDebugOut().bin.write(&data[0], size);
      Global::getDebugOut().finishMessage(idAnnotation);
      return true;
    }

    case idJPEGImage:
      if(Blackboard::getInstance().exists("CameraImage"))
      {
        JPEGImage jpegImage;
        message.bin >> jpegImage;
        jpegImage.toCameraImage(static_cast<CameraImage&>(Blackboard::getInstance()["CameraImage"]));
        if(Blackboard::getInstance().exists("FrameInfo"))
          static_cast<FrameInfo&>(Blackboard::getInstance()["FrameInfo"]).time = static_cast<const CameraImage&>(Blackboard::getInstance()["CameraImage"]).timestamp;
      }
      return true;

    default:
      return handle(message);
  }
}
