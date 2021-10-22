#include "TorsoMatrix.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Rotation.h"

void TorsoMatrix::setTorsoMatrix(const InertialData& theInertialData, const RobotModel& theRobotModel, const GroundContactState& theGroundContactState)
{
  const Vector3f axis(theInertialData.angle.x(), theInertialData.angle.y(), 0.0f);
  const RotationMatrix torsoRotation = Rotation::AngleAxis::unpack(axis);

  // calculate "center of hip" position from left foot
  Pose3f fromLeftFoot = Pose3f(torsoRotation) *= theRobotModel.soleLeft;
  fromLeftFoot.translation *= -1.;
  fromLeftFoot.rotation = torsoRotation;

  // calculate "center of hip" position from right foot
  Pose3f fromRightFoot = Pose3f(torsoRotation) *= theRobotModel.soleRight;
  fromRightFoot.translation *= -1.;
  fromRightFoot.rotation = torsoRotation;

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z() > fromRightFoot.translation.z();

  // calculate foot span
  const Vector3f newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3f newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x() / (useLeft ? 2.f : -2.f), newFootSpan.y() / (useLeft ? 2.f : -2.f), 0);
  newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);
  static_cast<Pose3f&>(*this) = newTorsoMatrix;

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
