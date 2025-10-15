/*
 * @file IMUCalibrationProvider.cpp
 * @author Philip Reichenberg
 */

#include "IMUCalibrationProvider.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Settings.h"
#include "Math/Eigen.h"
#include "Math/Rotation.h"
#include "Platform/File.h"
#include <cmath>
#include <filesystem>

MAKE_MODULE(IMUCalibrationProvider);

IMUCalibrationProvider::IMUCalibrationProvider()
{
  InMapFile stream("imuCalibration.cfg");
  ASSERT(stream.exists());
  stream >> const_cast<IMUCalibration&>(theIMUCalibration);
}

void IMUCalibrationProvider::update(IMUCalibration& imuCalibration)
{
  bool update = false;
  MODIFY("module:IMUCalibrationProvider:forceUpdate", update);
  if((theCalibrationRequest.serialNumberIMUCalibration > imuCalibration.serialNumberIMUCalibration && isStandingStill()))
  {
    const Vector3f accValues = imuCalibration.rotation.inverse() * theIMUValueState.accValues.mean;
    const Quaternionf imuAngles = Rotation::Euler::fromAngles(theRawInertialSensorData.angle.x(), theRawInertialSensorData.angle.y(), 0.f);
    const Quaternionf accQuat = Rotation::removeZRotation(Quaternionf::FromTwoVectors(accValues.normalized(), Vector3f(0.f, 0.f, 1.f)));
    imuCalibration.rotation = imuAngles.inverse() * accQuat;

    imuCalibration.serialNumberIMUCalibration = theCalibrationRequest.serialNumberIMUCalibration;
    imuCalibration.isCalibrated = true;
    //save the calibration
    const std::string path = std::string(File::getBHDir()) + "/Config/Robots/" + Global::getSettings().bodyName + "/Body";
    std::filesystem::create_directories(path);
    OutMapFile stream(path + "/imuCalibration.cfg", true);
    ASSERT(stream.exists());
    stream << imuCalibration;
  }
  else if((isAutoCalibrationActive && (update || isStandingStill()) && theFrameInfo.getTimeSince(lastCalibration) > minTimeBetweenCalibration))
  {
    const auto timeSinceCalibrationStarted = theFrameInfo.getTimeSince(calibrationStarted);
    if(timeSinceCalibrationStarted > waitTimeTillNewCalibrationAccepted && timeSinceCalibrationStarted < minStandStillTime)
    {
      calibrationStarted = 0;
      lastCalibration = theFrameInfo.time;
      imuCalibration.rotation = tempIMUCalibration.rotation;
      imuCalibration.isCalibrated = true;
    }
    else if(theFrameInfo.getTimeSince(calibrationStarted) > minStandStillTime)
    {
      calibrationStarted = theFrameInfo.time;
      tempIMUCalibration = imuCalibration;
      const Vector3f accValues = imuCalibration.rotation.inverse() * theIMUValueState.accValues.mean;
      const Quaternionf imuAngles = Rotation::Euler::fromAngles(theRawInertialSensorData.angle.x(), theRawInertialSensorData.angle.y(), 0.f);
      const Quaternionf accQuat = Rotation::removeZRotation(Quaternionf::FromTwoVectors(accValues.normalized(), Vector3f(0.f, 0.f, 1.f)));
      tempIMUCalibration.rotation = imuAngles.inverse() * accQuat;
    }
  }
  else
    calibrationStarted = 0;
}

bool IMUCalibrationProvider::isStandingStill()
{
  return theFrameInfo.getTimeSince(theIMUValueState.notMovingSinceTimestamp) > minStandStillTime &&
         theGroundContactState.contact &&
         std::abs(theRawInertialSensorData.angle.x()) < maxtorsoAngles.x() && std::abs(theRawInertialSensorData.angle.y()) < maxtorsoAngles.y();
}
