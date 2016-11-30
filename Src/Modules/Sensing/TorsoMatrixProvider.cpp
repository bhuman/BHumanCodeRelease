/**
 * @file TorsoMatrixProvider.cpp
 * Implementation of module TorsoMatrixProvider.
 * @author Colin Graf
 */

#include "TorsoMatrixProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Math/Rotation.h"

MAKE_MODULE(TorsoMatrixProvider, sensing)

void TorsoMatrixProvider::update(TorsoMatrix& torsoMatrix)
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

  // get foot z-rotations
  const Pose3f& leftFootInverse(theRobotModel.limbs[Limbs::footLeft].inverse());
  const Pose3f& rightFootInverse(theRobotModel.limbs[Limbs::footRight].inverse());
  const float leftFootZRotation = leftFootInverse.rotation.getZAngle();
  const float rightFootZRotation = rightFootInverse.rotation.getZAngle();

  // determine used foot
  const bool useLeft = fromLeftFoot.translation.z() > fromRightFoot.translation.z();
  torsoMatrix.leftSupportFoot = useLeft;

  // calculate foot span
  const Vector3f newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  // and construct the matrix
  Pose3f newTorsoMatrix;
  newTorsoMatrix.translate(newFootSpan.x() / (useLeft ? 2.f : -2.f), newFootSpan.y() / (useLeft ? 2.f : -2.f), 0);
  newTorsoMatrix.conc(useLeft ? fromLeftFoot : fromRightFoot);

  // calculate torso offset
  if(torsoMatrix.translation.z() != 0) // the last torso matrix should be valid
  {
    Pose3f& torsoOffset = torsoMatrix.offset;
    torsoOffset = torsoMatrix.inverse();
    torsoOffset.translate(lastFootSpan.x() / (useLeft ? 2.f : -2.f), lastFootSpan.y() / (useLeft ? 2.f : -2.f), 0);
    torsoOffset.rotateZ(useLeft ? float(leftFootZRotation - lastLeftFootZRotation) : float(rightFootZRotation - lastRightFootZRotation));
    torsoOffset.translate(newFootSpan.x() / (useLeft ? -2.f : 2.f), newFootSpan.y() / (useLeft ? -2.f : 2.f), 0);
    torsoOffset.conc(newTorsoMatrix);
  }

  // adopt new matrix and footSpan
  static_cast<Pose3f&>(torsoMatrix) = newTorsoMatrix;
  lastLeftFootZRotation = leftFootZRotation;
  lastRightFootZRotation = rightFootZRotation;
  lastFootSpan = newFootSpan;

  // valid?
  torsoMatrix.isValid = theGroundContactState.contact;

  //
  PLOT("module:TorsoMatrixProvider:footSpanX", newFootSpan.x());
  PLOT("module:TorsoMatrixProvider:footSpanY", newFootSpan.y());
  PLOT("module:TorsoMatrixProvider:footSpanZ", newFootSpan.z());
}

void TorsoMatrixProvider::update(OdometryData& odometryData)
{
  Pose2f odometryOffset;

  if(lastTorsoMatrix.translation.z() != 0.)
  {
    Pose3f odometryOffset3D(lastTorsoMatrix);
    odometryOffset3D.conc(theTorsoMatrix.offset);
    odometryOffset3D.conc(theTorsoMatrix.inverse());

    odometryOffset.translation.x() = odometryOffset3D.translation.x();
    odometryOffset.translation.y() = odometryOffset3D.translation.y();
    odometryOffset.rotation = odometryOffset3D.rotation.getZAngle();
  }

  PLOT("module:TorsoMatrixProvider:odometryOffsetX", odometryOffset.translation.x());
  PLOT("module:TorsoMatrixProvider:odometryOffsetY", odometryOffset.translation.y());
  PLOT("module:TorsoMatrixProvider:odometryOffsetRotation", odometryOffset.rotation.toDegrees());

  odometryData += odometryOffset;

  (Pose3f&)lastTorsoMatrix = theTorsoMatrix;
}
