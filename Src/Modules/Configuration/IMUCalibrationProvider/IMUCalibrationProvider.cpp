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
  MODIFY_ONCE("module:IMUCalibrationProvider:forceUpdate", update);
  if(update || (theCalibrationRequest.serialNumberIMUCalibration > imuCalibration.serialNumberIMUCalibration && isStandingStill()))
  {
    imuCalibration.rotation = Quaternionf(imuCalibration.rotation) *
                              (Rotation::Euler::fromAngles(-theRawInertialSensorData.angle.x() + theInertialData.angle.x(),
                                                           -theRawInertialSensorData.angle.y() + theInertialData.angle.y(), 0));
    imuCalibration.serialNumberIMUCalibration = theCalibrationRequest.serialNumberIMUCalibration;
    imuCalibration.isCalibrated = true;
    //save the calibration
    const std::string path = std::string(File::getBHDir()) + "/Config/Robots/" + Global::getSettings().bodyName + "/Body";
    std::filesystem::create_directories(path);
    OutMapFile stream(path + "/imuCalibration.cfg", true);
    ASSERT(stream.exists());
    stream << imuCalibration;
  }
  else if(isAutoCalibrationActive && isStandingStill() && theFrameInfo.getTimeSince(lastCalibration) > minTimeBetweenCalibration)
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
      const Vector3f accAngles = Rotation::AngleAxis::pack(AngleAxisf(Rotation::removeZRotation(Quaternionf::FromTwoVectors(theIMUValueState.accValues.mean.normalized(), Vector3f(0.f, 0.f, 1.f)))));
      tempIMUCalibration.rotation = (Rotation::Euler::fromAngles(-theRawInertialSensorData.angle.x() + accAngles.x(),
                                                                 -theRawInertialSensorData.angle.y() + accAngles.y(), 0));
    }
  }
  else
    calibrationStarted = 0;
}

bool IMUCalibrationProvider::isStandingStill()
{
  return theFrameInfo.getTimeSince(theIMUValueState.notMovingSinceTimestamp) > minStandStillTime && theGroundContactState.contact;
}
