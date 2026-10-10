#include "InertialSensorData.h"
#include "SystemSensorData.h"
#include "Debugging/Plot.h"

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
  PLOT("representation:SystemSensorData:batteryLevel", batteryLevel);
}
