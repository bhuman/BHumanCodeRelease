#include "TorsoMatrix.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Math/Rotation.h"

void TorsoMatrix::setTorsoMatrix(const InertialData& theInertialData, const RobotModel& theRobotModel, const GroundContactState& theGroundContactState)
{
  const Vector3f axis(theInertialData.angle.x(), theInertialData.angle.y(), 0.0f);
  const RotationMatrix torsoRotation = Rotation::AngleAxis::unpack(axis);

  // calculate "center of hip" position from left foot
  const Vector3f fromLeftFoot = -torsoRotation * theRobotModel.soleLeft.translation;

  // calculate "center of hip" position from right foot
  const Vector3f fromRightFoot = -torsoRotation * theRobotModel.soleRight.translation;

  // and construct the matrix
  translation << 0.5f * (fromLeftFoot.head<2>() + fromRightFoot.head<2>()), std::max(fromLeftFoot.z(), fromRightFoot.z());
  rotation = torsoRotation;

  covariance << Matrix3f::Zero(), Matrix3f::Zero(), Matrix3f::Zero(), theInertialData.orientation3DCov;

  // valid?
  isValid = theGroundContactState.contact;
}

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
    CYLINDERARROW3D("representation:TorsoMatrix:coordinateSystem", Vector3f::Zero(), Vector3f(200.f, 0.f, 0.f), 5.f, 35.f, 15.f, ColorRGBA::red);
    CYLINDERARROW3D("representation:TorsoMatrix:coordinateSystem", Vector3f::Zero(), Vector3f(0.f, 200.f, 0.f), 5.f, 35.f, 15.f, ColorRGBA::green);
    CYLINDERARROW3D("representation:TorsoMatrix:coordinateSystem", Vector3f::Zero(), Vector3f(0.f, 0.f, 200.f), 5.f, 35.f, 15.f, ColorRGBA::blue);
  }
}
