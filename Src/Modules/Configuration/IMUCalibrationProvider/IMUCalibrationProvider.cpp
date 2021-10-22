/*
 * @file IMUCalibrationProvider.cpp
 * @author Philip Reichenberg
 */

#include "IMUCalibrationProvider.h"
#include "Platform/File.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Rotation.h"
#include <cmath>

MAKE_MODULE(IMUCalibrationProvider, sensing);

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
  if(update || theCalibrationRequest.serialNumberIMUCalibration > serialNumberIMUCalibration)
  {
    imuCalibration.rotation = Rotation::Euler::fromAngles(-theInertialSensorData.angle.x() + theInertialData.angle.x(), -theInertialSensorData.angle.y() + theInertialData.angle.y(), 0);
    serialNumberIMUCalibration = theCalibrationRequest.serialNumberIMUCalibration;
    imuCalibration.isCalibrated = true;
    //save the calibration
    std::string name = "imuCalibration.cfg";
    for(std::string& fullName : File::getFullNames(name))
    {
      File path(fullName, "r", false);
      if(path.exists())
      {
        name = std::move(fullName);
        break;
      }
    }
    OutMapFile stream(name, true);
    ASSERT(stream.exists());
    stream << imuCalibration;
  }
}
