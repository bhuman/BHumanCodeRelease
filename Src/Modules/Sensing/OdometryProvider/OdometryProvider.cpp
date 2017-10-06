/**
 * @file OdometryProvider.cpp
 *
 * This file implements a module that calculates walking engine odometry.
 *
 * @author Jonas Kuball
 */

#include "OdometryProvider.h"
#include "Tools/Math/Rotation.h"

MAKE_MODULE(OdometryProvider, sensing)

OdometryProvider::OdometryProvider() : torsoAngleEstimator(theInertialSensorData, theRobotModel, theMotionRequest)
{
}

void OdometryProvider::update(OdometryOffset& theOdometryOffset)
{
  Pose2f odometryOffset = computeOdometryOffset(computeTorsoMatrix());
  odometry += odometryOffset;

  Pose2f combinedOdometry = odometry;
  for(const Pose2f& forwardOffset : forwardOdometryOffsets)
    combinedOdometry += forwardOffset;

  static_cast<Pose2f&>(theOdometryOffset) = combinedOdometry - lastCombinedOdometry;
  lastCombinedOdometry = combinedOdometry;

  swingPhases.push_front(theWalkGenerator.isLeftPhase);
  forwardOdometryOffsets.push_front(theWalkGenerator.odometryOffset);
}

Pose3f OdometryProvider::computeTorsoMatrix()
{
  Vector2a torsoAngle = torsoAngleEstimator.update(theWalkGenerator.speed.translation.x());
  MODIFY("torsoAngle", torsoAngle);

  const Vector3f axis(torsoAngle.x(), torsoAngle.y(), 0.f);
  const RotationMatrix torsoRotation = Rotation::AngleAxis::unpack(axis);

  // calculate "center of hip" position from left foot
  Pose3f fromLeftFoot = Pose3f(torsoRotation) *= theRobotModel.soleLeft;
  fromLeftFoot.translation *= -1.f;
  fromLeftFoot.rotation = torsoRotation;

  // calculate "center of hip" position from right foot
  Pose3f fromRightFoot = Pose3f(torsoRotation) *= theRobotModel.soleRight;
  fromRightFoot.translation *= -1.f;
  fromRightFoot.rotation = torsoRotation;

  const bool useLeft = fromLeftFoot.translation.z() > fromRightFoot.translation.z(); // determine used foot
  const Vector3f newFootSpan(fromRightFoot.translation - fromLeftFoot.translation); // calculate foot span
  return Pose3f(newFootSpan.x() / (useLeft ? 2.f : -2.f), newFootSpan.y() / (useLeft ? 2.f : -2.f), 0).conc(useLeft ? fromLeftFoot : fromRightFoot); // and construct the matrix
}

Pose2f OdometryProvider::computeOdometryOffset(const Pose3f& torsoMatrix)
{
  Pose2f odometryOffset;
  Vector3f odometryOrigin = (theRobotModel.soleLeft.translation + theRobotModel.soleRight.translation) * 0.5f;

  if(lastOdometryOrigin.z() != 0.f)
  {
    const Pose3f& footSupport = swingPhases.back() ? theRobotModel.soleRight : theRobotModel.soleLeft;
    const Pose3f& lastFootSupport = swingPhases.back() ? lastFootRight : lastFootLeft;
    Pose3f odometryOffset3DinP = (Pose3f(-odometryOrigin).conc(footSupport).conc(lastFootSupport.inverse()).conc(lastOdometryOrigin)).inverse();
    Pose3f odometryOffset3D = Pose3f(torsoMatrix).conc(odometryOffset3DinP).conc(torsoMatrix.inverse());

    odometryOffset.rotation = odometryOffset3D.rotation.getZAngle() * odometryScale.rotation;
    odometryOffset.translation = (odometryOffset3D.translation.array().head<2>() * odometryScale.translation.array());
  }

  lastFootLeft = theRobotModel.soleLeft;
  lastFootRight = theRobotModel.soleRight;
  lastOdometryOrigin = odometryOrigin;

  return odometryOffset;
}
