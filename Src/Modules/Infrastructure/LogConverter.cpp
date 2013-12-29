/**
 * @file LogConverter.cpp
 *
 * @author reich
 * @author <a href="afabisch@tzi.de>Alexander Fabisch</a>
 */

#include <cstring>

#include "LogDataProvider.h"
#include "LogConverter.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/Legacy/SensorDataRev5703.h"
#include "Representations/Infrastructure/Legacy/SensorData_2d284.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Infrastructure/Legacy/JointData_d198df791237.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Legacy/FrameInfo_83e22.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Tools/Streams/OutStreams.h"

LogConverter::LogConverter() :
  sizeofSensorDataRev5703(sizeofRepresentation(SensorDataRev5703())),
  sizeofSensorData_2d284(sizeofRepresentation(SensorData_2d284())),
  sizeofFrameInfo_83e22(sizeofRepresentation(FrameInfo_83e22())),
  sizeofJointData_d198df791237(sizeofRepresentation(JointData_d198df791237()))
{
}

std::size_t LogConverter::sizeofRepresentation(const Streamable& streamable)
{
  OutBinarySize outSize;
  outSize << streamable;
  return outSize.getSize();
}

Streamable* LogConverter::newConvertedRepresentation(InMessage& message, const int representationId)
{
  switch(representationId)
  {
  case idSensorData:
  case idFilteredSensorData:
    return newConvertedSensorData(message);
  case idFrameInfo:
    return newConvertedFrameInfo(message);
  case idJointData:
  case idFilteredJointData:
    return newConvertedJointData(message);
  case idCameraInfo:
    return newConvertedCameraInfo(message);
  default:
    ASSERT(false);
  }
  return 0;
}

bool LogConverter::isConversionRequired(InMessage& message, const int representationId)
{
  switch(representationId)
  {
  case idSensorData:
  case idFilteredSensorData:
    return (std::size_t)message.getMessageSize() == sizeofSensorDataRev5703 ||
           (std::size_t)message.getMessageSize() == sizeofSensorData_2d284;
  case idFrameInfo:
    return (std::size_t)message.getMessageSize() == sizeofFrameInfo_83e22;
  case idJointData:
  case idFilteredJointData:
    return (std::size_t)message.getMessageSize() == sizeofJointData_d198df791237;
  case idCameraInfo:
    return (std::size_t)message.getMessageSize() == 56; // very old logfiles
  default:
    return false;
  }
}

Streamable* LogConverter::newConvertedSensorData(InMessage& message)
{
  SensorData* sensorData = new SensorData;
  if((std::size_t)message.getMessageSize() == sizeofSensorDataRev5703)
  {
    SensorDataRev5703 sensorDataRev5703;
    message.bin >> sensorDataRev5703;
    message.resetReadPosition();

    memcpy(sensorData->data, sensorDataRev5703.data, sizeof(float) * SensorDataRev5703::us);
    memcpy(&sensorData->data[SensorData::angleX], &sensorDataRev5703.data[SensorDataRev5703::angleX],
           sizeof(float) * (SensorData::numOfSensors - SensorData::angleX));
    memcpy(sensorData->currents, sensorDataRev5703.data, sizeof(sensorData->currents));
    memcpy(sensorData->temperatures, sensorDataRev5703.temperatures, sizeof(sensorData->temperatures));

    sensorData->timeStamp = sensorDataRev5703.timeStamp;
    sensorData->usActuatorMode = (SensorData::UsActuatorMode) sensorDataRev5703.usSensorType;
    sensorData->usTimeStamp = sensorData->timeStamp;
  }
  else
  {
    SensorData_2d284 sensorData_2d284;
    message.bin >> sensorData_2d284;
    message.resetReadPosition();

    memcpy(sensorData->data, sensorData_2d284.data, sizeof(float) * SensorData_2d284::usR);
    sensorData->data[SensorData::usR] = sensorData_2d284.data[SensorData_2d284::usR];
    sensorData->data[SensorData::angleX] = sensorData_2d284.data[SensorData_2d284::angleX];
    sensorData->data[SensorData::angleY] = sensorData_2d284.data[SensorData_2d284::angleY];

    for(int i = SensorData::usL1; i < SensorData::usLEnd; ++i)
      sensorData->data[i] = 2550.f;
    for(int i = SensorData::usR1; i < SensorData::usREnd; ++i)
      sensorData->data[i] = 2550.f;

    sensorData->timeStamp = sensorData_2d284.timeStamp;
    sensorData->usActuatorMode = (SensorData::UsActuatorMode) sensorData_2d284.usActuatorMode;
    sensorData->usTimeStamp = sensorData_2d284.usTimeStamp;
  }

  return sensorData;
}

Streamable* LogConverter::newConvertedFrameInfo(InMessage& message)
{
  FrameInfo_83e22 frameInfo_83e22;
  FrameInfo* frameInfo = new FrameInfo;
  message.bin >> frameInfo_83e22;
  message.resetReadPosition();

  frameInfo->time = frameInfo_83e22.time;
  frameInfo->cycleTime = 0.01f;

  return frameInfo;
}

Streamable* LogConverter::newConvertedJointData(InMessage& message)
{
  JointData_d198df791237 jointData_d198df791237;
  JointData* jointData = new JointData;
  message.bin >> jointData_d198df791237;
  message.resetReadPosition();

  memcpy(jointData->angles, jointData_d198df791237.angles, sizeof(float) * JointData::numOfJoints);
  jointData->timeStamp = jointData_d198df791237.timeStamp;

  return jointData;
}

Streamable* LogConverter::newConvertedCameraInfo(InMessage& message)
{
  char buf[100];
  OutBinaryMemory out(buf);
  out << CameraInfo::lower << 320 << 240 << 0.78674f << 0.60349f << 160.f << 120.f;

  CameraInfo* cameraInfo = new CameraInfo;
  InBinaryMemory in(buf);
  in >> *cameraInfo;
  return cameraInfo;
}
