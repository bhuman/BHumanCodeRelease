#include "FsrSensorData.h"
#include "RawInertialSensorData.h"
#include "SystemSensorData.h"
#include "Debugging/Plot.h"

void FsrSensorData::draw()
{
  PLOT("representation:FsrSensorData:leftFrontLeft", pressures[Legs::left][FsrSensors::fl]);
  PLOT("representation:FsrSensorData:leftFrontRight", pressures[Legs::left][FsrSensors::fr]);
  PLOT("representation:FsrSensorData:leftBackLeft", pressures[Legs::left][FsrSensors::bl]);
  PLOT("representation:FsrSensorData:leftBackRight", pressures[Legs::left][FsrSensors::br]);
  PLOT("representation:FsrSensorData:rightFrontLeft", pressures[Legs::right][FsrSensors::fl]);
  PLOT("representation:FsrSensorData:rightFrontRight", pressures[Legs::right][FsrSensors::fr]);
  PLOT("representation:FsrSensorData:rightBackLeft", pressures[Legs::right][FsrSensors::bl]);
  PLOT("representation:FsrSensorData:rightBackRight", pressures[Legs::right][FsrSensors::br]);
  PLOT("representation:FsrSensorData:leftTotal", totals[Legs::left]);
  PLOT("representation:FsrSensorData:rightTotal", totals[Legs::right]);
}

void RawInertialSensorData::draw()
{
  PLOT("representation:RawInertialSensorData:gyro:x", gyro.x().toDegrees());
  PLOT("representation:RawInertialSensorData:gyro:y", gyro.y().toDegrees());
  PLOT("representation:RawInertialSensorData:gyro:z", gyro.z().toDegrees());
  PLOT("representation:RawInertialSensorData:acc:x", acc.x());
  PLOT("representation:RawInertialSensorData:acc:y", acc.y());
  PLOT("representation:RawInertialSensorData:acc:z", acc.z());
  PLOT("representation:RawInertialSensorData:angle:x", angle.x().toDegrees());
  PLOT("representation:RawInertialSensorData:angle:y", angle.y().toDegrees());
}

void SystemSensorData::draw()
{
  PLOT("representation:SystemSensorData:cpuTemperature", cpuTemperature);
  PLOT("representation:SystemSensorData:batteryCurrent", batteryCurrent);
  PLOT("representation:SystemSensorData:batteryLevel", batteryLevel);
  PLOT("representation:SystemSensorData:batteryTemperature", batteryTemperature);
  PLOT("representation:SystemSensorData:batteryCharging", batteryCharging);
}
