#include "InertialData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Rotation.h"

void InertialData::draw()
{
  PLOT("representation:InertialData:gyro:x", gyro.x().toDegrees());
  PLOT("representation:InertialData:gyro:y", gyro.y().toDegrees());
  PLOT("representation:InertialData:gyro:z", gyro.z().toDegrees());
  PLOT("representation:InertialData:acc:x", acc.x());
  PLOT("representation:InertialData:acc:y", acc.y());
  PLOT("representation:InertialData:acc:z", acc.z());
  PLOT("representation:InertialData:angle:x", angle.x().toDegrees());
  PLOT("representation:InertialData:angle:y", angle.y().toDegrees());

  const Vector3a oreintationVec = Rotation::AngleAxis::pack(AngleAxisf(orientation)).cast<Angle>();
  PLOT("representation:InertialData:orientation:x", oreintationVec.x().toDegrees());
  PLOT("representation:InertialData:orientation:y", oreintationVec.y().toDegrees());
  PLOT("representation:InertialData:orientation:z", oreintationVec.z().toDegrees());
}
