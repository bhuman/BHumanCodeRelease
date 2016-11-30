#include "FsrSensorData.h"
#include "InertialSensorData.h"
#include "SystemSensorData.h"
#include "Tools/Debugging/DebugDrawings.h"

void FsrSensorData::draw()
{
  PLOT("representation:FsrSensorData:leftFrontLeft", left[FsrSensors::fl]);
  PLOT("representation:FsrSensorData:leftFrontRight", left[FsrSensors::fr]);
  PLOT("representation:FsrSensorData:leftBackLeft", left[FsrSensors::bl]);
  PLOT("representation:FsrSensorData:leftBackRight", left[FsrSensors::br]);
  PLOT("representation:FsrSensorData:rightFrontLeft", right[FsrSensors::fl]);
  PLOT("representation:FsrSensorData:rightFrontRight", right[FsrSensors::fr]);
  PLOT("representation:FsrSensorData:rightBackLeft", right[FsrSensors::bl]);
  PLOT("representation:FsrSensorData:rightBackRight", right[FsrSensors::br]);
  PLOT("representation:FsrSensorData:leftTotal", leftTotal);
  PLOT("representation:FsrSensorData:rightTotal", rightTotal);
}

void InertialSensorData::draw()
{
  PLOT("representation:InertialSensorData:gyro:x", gyro.x().toDegrees());
  PLOT("representation:InertialSensorData:gyro:y", gyro.y().toDegrees());
  PLOT("representation:InertialSensorData:gyro:z", gyro.z().toDegrees());
  PLOT("representation:InertialSensorData:acc:x", acc.x());
  PLOT("representation:InertialSensorData:acc:y", acc.y());
  PLOT("representation:InertialSensorData:acc:z", acc.z());
  PLOT("representation:InertialSensorData:angle:x", angle.x().toDegrees());
  PLOT("representation:InertialSensorData:angle:y", angle.y().toDegrees());
}

void SystemSensorData::draw()
{
  PLOT("representation:SystemSensorData:cpuTemperature", cpuTemperature);
  PLOT("representation:SystemSensorData:batteryCurrent", batteryCurrent);
  PLOT("representation:SystemSensorData:batteryLevel", batteryLevel);
  PLOT("representation:SystemSensorData:batteryTemperature", batteryTemperature);
  PLOT("representation:SystemSensorData:batteryCharging", batteryCharging);
}
