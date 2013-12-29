/**
* @file SensorFilter.cpp
* Implementation of module SensorFilter.
* @author Colin Graf
*/

#include "SensorFilter.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(SensorFilter, Sensing)

void SensorFilter::update(FilteredSensorData& filteredSensorData)
{
  // copy sensor data from representation SensorData to representation FilteredSensorData,
  // but keep values from gyro and acc
  Vector2<> gyro(filteredSensorData.data[SensorData::gyroX], filteredSensorData.data[SensorData::gyroY]);
  Vector3<> acc(filteredSensorData.data[SensorData::accX], filteredSensorData.data[SensorData::accY], filteredSensorData.data[SensorData::accZ]);
  (SensorData&)filteredSensorData = theSensorData;
  filteredSensorData.data[SensorData::gyroX] = gyro.x;
  filteredSensorData.data[SensorData::gyroY] = gyro.y;
  filteredSensorData.data[SensorData::accX] = acc.x;
  filteredSensorData.data[SensorData::accY] = acc.y;
  filteredSensorData.data[SensorData::accZ] = acc.z;

  if(theDamageConfiguration.usLDefect)
  {
    for(int i = SensorData::usL; i < SensorData::usLEnd; ++i)
      filteredSensorData.data[i] = 2550.0f;
  }

  if(theDamageConfiguration.usRDefect)
  {
    for(int i = SensorData::usR; i < SensorData::usREnd; ++i)
      filteredSensorData.data[i] = 2550.0f;
  }

  // take calibrated inertia sensor data
  for(int i = 0; i < 2; ++i)
  {
    if(theInertiaSensorData.gyro[i] != InertiaSensorData::off)
      filteredSensorData.data[SensorData::gyroX + i] = theInertiaSensorData.gyro[i];
    else if(filteredSensorData.data[SensorData::gyroX + i] == SensorData::off)
      filteredSensorData.data[SensorData::gyroX + i] = 0.f;
  }
  filteredSensorData.data[SensorData::gyroZ] = 0.f;
  for(int i = 0; i < 3; ++i)
  {
    if(theInertiaSensorData.acc[i] != InertiaSensorData::off)
      filteredSensorData.data[SensorData::accX + i] = theInertiaSensorData.acc[i];
    else if(filteredSensorData.data[SensorData::accX + i] == SensorData::off)
      filteredSensorData.data[SensorData::accX + i] = 0.f;
  }

  // take orientation data
  filteredSensorData.data[SensorData::angleX] = std::atan2(theOrientationData.rotation.c1.z, theOrientationData.rotation.c2.z);
  filteredSensorData.data[SensorData::angleY] = std::atan2(-theOrientationData.rotation.c0.z, theOrientationData.rotation.c2.z);

  // some code for calibrating the gain of the gyro sensors:
#ifndef RELEASE
  if(filteredSensorData.data[SensorData::gyroX] != SensorData::off)
  {
    gyroAngleXSum += filteredSensorData.data[SensorData::gyroX] * (theSensorData.timeStamp - lastIteration) * 0.001f;
    gyroAngleXSum = normalize(gyroAngleXSum);
    lastIteration = theSensorData.timeStamp;
  }
  PLOT("module:SensorFilter:gyroAngleXSum", gyroAngleXSum);
  DEBUG_RESPONSE_ONCE("module:SensorFilter:gyroAngleXSum:reset", gyroAngleXSum = 0.f;);
#endif

  PLOT("module:SensorFilter:rawAngleX", theSensorData.data[SensorData::angleX]);
  PLOT("module:SensorFilter:rawAngleY", theSensorData.data[SensorData::angleY]);

  PLOT("module:SensorFilter:rawAccX", theSensorData.data[SensorData::accX]);
  PLOT("module:SensorFilter:rawAccY", theSensorData.data[SensorData::accY]);
  PLOT("module:SensorFilter:rawAccZ", theSensorData.data[SensorData::accZ]);

  PLOT("module:SensorFilter:rawGyroX", theSensorData.data[SensorData::gyroX]);
  PLOT("module:SensorFilter:rawGyroY", theSensorData.data[SensorData::gyroY]);
  PLOT("module:SensorFilter:rawGyroZ", theSensorData.data[SensorData::gyroZ]);

  PLOT("module:SensorFilter:angleX", filteredSensorData.data[SensorData::angleX]);
  PLOT("module:SensorFilter:angleY", filteredSensorData.data[SensorData::angleY]);

  PLOT("module:SensorFilter:accX", filteredSensorData.data[SensorData::accX]);
  PLOT("module:SensorFilter:accY", filteredSensorData.data[SensorData::accY]);
  PLOT("module:SensorFilter:accZ", filteredSensorData.data[SensorData::accZ]);

  PLOT("module:SensorFilter:gyroX", filteredSensorData.data[SensorData::gyroX] != float(SensorData::off) ? filteredSensorData.data[SensorData::gyroX] : 0);
  PLOT("module:SensorFilter:gyroY", filteredSensorData.data[SensorData::gyroY] != float(SensorData::off) ? filteredSensorData.data[SensorData::gyroY] : 0);
  PLOT("module:SensorFilter:gyroZ", filteredSensorData.data[SensorData::gyroZ] != float(SensorData::off) ? filteredSensorData.data[SensorData::gyroZ] : 0);
}

