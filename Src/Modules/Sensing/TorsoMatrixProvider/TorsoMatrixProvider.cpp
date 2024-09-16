/**
 * @file TorsoMatrixProvider.cpp
 * Implementation of module TorsoMatrixProvider.
 * @author Colin Graf
 */

#include "TorsoMatrixProvider.h"
#include "Math/Rotation.h"

MAKE_MODULE(TorsoMatrixProvider);

void TorsoMatrixProvider::update(TorsoMatrix& torsoMatrix)
{
  const Vector3f axis(theInertialData.angle.x(), theInertialData.angle.y(), 0.0f);
  const RotationMatrix torsoRotation = Rotation::AngleAxis::unpack(axis);

  // calculate "center of hip" position from left foot
  const Vector3f fromLeftFoot = -torsoRotation * theRobotModel.soleLeft.translation;

  // calculate "center of hip" position from right foot
  const Vector3f fromRightFoot = -torsoRotation * theRobotModel.soleRight.translation;

  // and construct the matrix
  torsoMatrix.translation << 0.5f * (fromLeftFoot.head<2>() + fromRightFoot.head<2>()), std::max(fromLeftFoot.z(), fromRightFoot.z());
  torsoMatrix.rotation = torsoRotation;

  torsoMatrix.covariance << Matrix3f::Zero(), Matrix3f::Zero(), Matrix3f::Zero(), theInertialData.orientation3DCov;

  // valid?
  torsoMatrix.isValid = theGroundContactState.contact;
}
