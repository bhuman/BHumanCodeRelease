#include "InertialData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
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

  PLOT("representation:InertialData:filteredAcc:x", filteredAcc.x());
  PLOT("representation:InertialData:filteredAcc:y", filteredAcc.y());
  PLOT("representation:InertialData:filteredAcc:z", filteredAcc.z());

  const Vector3a orientation2DVec = Rotation::AngleAxis::pack(AngleAxisf(orientation2D)).cast<Angle>();
  PLOT("representation:InertialData:orientation2D:x", orientation2DVec.x().toDegrees());
  PLOT("representation:InertialData:orientation2D:y", orientation2DVec.y().toDegrees());

  const Vector3a orientation3DVec = Rotation::AngleAxis::pack(AngleAxisf(orientation3D)).cast<Angle>();
  PLOT("representation:InertialData:orientation3D:x", orientation3DVec.x().toDegrees());
  PLOT("representation:InertialData:orientation3D:y", orientation3DVec.y().toDegrees());
  PLOT("representation:InertialData:orientation3D:z", orientation3DVec.z().toDegrees());

  DEBUG_DRAWING3D("representation:InertialData:down", "robot")
  {
    const Vector3f zRotated = orientation3D.inverse() * Vector3f(0.f, 0.f, -1000.f);
    CYLINDERARROW3D("representation:InertialData:down", Vector3f::Zero(), zRotated, 5.f, 35.f, 35.f, ColorRGBA::blue);
  }
}
