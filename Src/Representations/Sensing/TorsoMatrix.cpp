#include "TorsoMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"

void TorsoMatrix::draw()
{
  PLOT("representation:TorsoMatrix:translation:x", translation.x());
  PLOT("representation:TorsoMatrix:translation:y", translation.y());
  PLOT("representation:TorsoMatrix:translation:z", translation.z());

  PLOT("representation:TorsoMatrix:rotation:x", toDegrees(rotation.getXAngle()));
  PLOT("representation:TorsoMatrix:rotation:y", toDegrees(rotation.getYAngle()));
  PLOT("representation:TorsoMatrix:rotation:z", toDegrees(rotation.getZAngle()));
}
