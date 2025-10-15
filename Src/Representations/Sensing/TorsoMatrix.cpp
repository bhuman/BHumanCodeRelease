#include "TorsoMatrix.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"

void TorsoMatrix::draw()
{
  PLOT("representation:TorsoMatrix:translation:x", translation.x());
  PLOT("representation:TorsoMatrix:translation:y", translation.y());
  PLOT("representation:TorsoMatrix:translation:z", translation.z());

  PLOT("representation:TorsoMatrix:rotation:x", toDegrees(rotation.getXAngle()));
  PLOT("representation:TorsoMatrix:rotation:y", toDegrees(rotation.getYAngle()));
  PLOT("representation:TorsoMatrix:rotation:z", toDegrees(rotation.getZAngle()));

  DEBUG_DRAWING3D("representation:TorsoMatrix:coordinateSystem", "robot")
  {
    CYLINDER_ARROW3D("representation:TorsoMatrix:coordinateSystem", Vector3f::Zero(), Vector3f(200.f, 0.f, 0.f), 5.f, 35.f, 15.f, ColorRGBA::red);
    CYLINDER_ARROW3D("representation:TorsoMatrix:coordinateSystem", Vector3f::Zero(), Vector3f(0.f, 200.f, 0.f), 5.f, 35.f, 15.f, ColorRGBA::green);
    CYLINDER_ARROW3D("representation:TorsoMatrix:coordinateSystem", Vector3f::Zero(), Vector3f(0.f, 0.f, 200.f), 5.f, 35.f, 15.f, ColorRGBA::blue);
  }
}
