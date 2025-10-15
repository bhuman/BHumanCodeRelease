/**
 * @file BoosterSolePressureStateProvider.cpp
 * This modules provides fake FSR data based on the kinematic and estimated torso orientation
 * @author Philip Reichenberg
 */

#include "BoosterSolePressureStateProvider.h"

MAKE_MODULE(BoosterSolePressureStateProvider);

void BoosterSolePressureStateProvider::update(FsrSensorData& theFsrSensorData)
{
  float leftTorque = 0.f;
  float rightTorque = 0.f;

  for(std::size_t joint = Joints::firstLeftLegJoint; joint < Joints::firstRightLegJoint; joint++)
  {
    if(joint == Joints::lHipRoll)
      continue;
    const float torque = std::abs(theJointSensorData.currents[joint] * torqueConversion);
    if(joint == Joints::lAnklePitch || joint == Joints::lAnkleRoll)
      leftTorque += torque - ankleTorqueClipRange.limit(torque);
    else
      leftTorque += torque;
  }
  for(std::size_t joint = Joints::firstRightLegJoint; joint < Joints::numOfJoints; joint++)
  {
    if(joint == Joints::rHipRoll)
      continue;
    const float torque = std::abs(theJointSensorData.currents[joint] * torqueConversion);
    if(joint == Joints::rAnklePitch || joint == Joints::rAnkleRoll)
      rightTorque += torque - ankleTorqueClipRange.limit(torque);
    else
      rightTorque += torque;
  }

  const Rangef& useScaleRange = theMotionInfo.isMotion(MotionPhase::walk) ? walkTorqueGroundContactScaleRange : standTorqueGroundContactScaleRange;

  const float maxTorquePerLeg = std::max(leftTorque, rightTorque);
  const float minTorqueGroundContactScaleRange = std::max(maxTorquePerLeg * torqueGroundContactPercentRange.min, useScaleRange.min);
  const float maxTorqueGroundContactScaleRange = std::max(maxTorquePerLeg * torqueGroundContactPercentRange.max, useScaleRange.max);

  const float groundOffset = mapToRange(std::max(rightTorque, leftTorque), useScaleRange.min, useScaleRange.max, 1.f, 0.f) * maxDistanceToGround;
  const float leftGroundOffset = mapToRange(theMotionInfo.isMotion(MotionPhase::walk) ? leftTorque : std::max(leftTorque, rightTorque), minTorqueGroundContactScaleRange, maxTorqueGroundContactScaleRange, removeGroundDistanceByTorque.min, removeGroundDistanceByTorque.max);
  const float rightGroundOffset = mapToRange(theMotionInfo.isMotion(MotionPhase::walk) ? rightTorque : std::max(leftTorque, rightTorque), minTorqueGroundContactScaleRange, maxTorqueGroundContactScaleRange, removeGroundDistanceByTorque.min, removeGroundDistanceByTorque.max);

  const Vector3f comInTorso = theTorsoMatrix * theRobotModel.centerOfMass;
  const Pose3f leftSoleInGround = theTorsoMatrix * theRobotModel.soleLeft;
  const Pose3f rightSoleInGround = theTorsoMatrix * theRobotModel.soleRight;

  const float length = leftSoleInGround.translation.y() - rightSoleInGround.translation.y();
  const float leftRatio = 1.f - Rangef::ZeroOneRange().limit((leftSoleInGround.translation.y() - comInTorso.y()) / length);
  const float rightRatio = 1.f - Rangef::ZeroOneRange().limit((comInTorso.y() - rightSoleInGround.translation.y()) / length);

  FSRPoint soleFSR[FsrSensors::numOfFsrSensors * 2];
  soleFSR[FsrSensors::fl] = { leftSoleInGround * Vector3f(theRobotDimensions.leftFsrPositions[FsrSensors::fl].x(), theRobotDimensions.leftFsrPositions[FsrSensors::fl].y(), 0.f), FsrSensors::fl, Legs::left };
  soleFSR[FsrSensors::fr] = { leftSoleInGround * Vector3f(theRobotDimensions.leftFsrPositions[FsrSensors::fr].x(), theRobotDimensions.leftFsrPositions[FsrSensors::fr].y(), 0.f), FsrSensors::fr, Legs::left };
  soleFSR[FsrSensors::bl] = { leftSoleInGround * Vector3f(theRobotDimensions.leftFsrPositions[FsrSensors::bl].x(), theRobotDimensions.leftFsrPositions[FsrSensors::bl].y(), 0.f), FsrSensors::bl, Legs::left };
  soleFSR[FsrSensors::br] = { leftSoleInGround * Vector3f(theRobotDimensions.leftFsrPositions[FsrSensors::br].x(), theRobotDimensions.leftFsrPositions[FsrSensors::br].y(), 0.f), FsrSensors::br, Legs::left };

  soleFSR[FsrSensors::numOfFsrSensors + FsrSensors::fl] = { rightSoleInGround * Vector3f(theRobotDimensions.rightFsrPositions[FsrSensors::fl].x(), theRobotDimensions.rightFsrPositions[FsrSensors::fl].y(), 0.f), FsrSensors::fl, Legs::right };
  soleFSR[FsrSensors::numOfFsrSensors + FsrSensors::fr] = { rightSoleInGround * Vector3f(theRobotDimensions.rightFsrPositions[FsrSensors::fr].x(), theRobotDimensions.rightFsrPositions[FsrSensors::fr].y(), 0.f), FsrSensors::fr, Legs::right };
  soleFSR[FsrSensors::numOfFsrSensors + FsrSensors::bl] = { rightSoleInGround * Vector3f(theRobotDimensions.rightFsrPositions[FsrSensors::bl].x(), theRobotDimensions.rightFsrPositions[FsrSensors::bl].y(), 0.f), FsrSensors::bl, Legs::right };
  soleFSR[FsrSensors::numOfFsrSensors + FsrSensors::br] = { rightSoleInGround * Vector3f(theRobotDimensions.rightFsrPositions[FsrSensors::br].x(), theRobotDimensions.rightFsrPositions[FsrSensors::br].y(), 0.f), FsrSensors::br, Legs::right };

  int len = sizeof(soleFSR) / sizeof(soleFSR[0]);
  std::sort(soleFSR, soleFSR + len, [](const FSRPoint& a, const FSRPoint& b) { return a.point.z() < b.point.z(); });

  const float groundPoint = soleFSR[0].point.z() - groundOffset;

  float weightRatio = 0.f;
  for(const FSRPoint& fsrPoint : soleFSR)
    weightRatio += (fsrPoint.leg == Legs::left ? leftRatio : rightRatio) * mapToRange(fsrPoint.point.z() - (fsrPoint.leg == Legs::left ? leftGroundOffset : rightGroundOffset), groundPoint, groundPoint + maxDistanceToGround, 1.f, 0.f);

  theFsrSensorData.totals[Legs::left] = 0.f;
  theFsrSensorData.totals[Legs::right] = 0.f;

  const float totalMass = theMassCalibration.totalMass / 1000.f;
  for(const FSRPoint& fsrPoint : soleFSR)
  {
    const float weight = weightRatio == 0.f ? 0.f : (fsrPoint.leg == Legs::left ? leftRatio : rightRatio) * mapToRange(fsrPoint.point.z() - (fsrPoint.leg == Legs::left ? leftGroundOffset : rightGroundOffset), groundPoint, groundPoint + maxDistanceToGround, 1.f, 0.f) / weightRatio * (totalMass * std::min(1.f, weightRatio));
    theFsrSensorData.pressures[fsrPoint.leg][fsrPoint.sensor] = weight;
    theFsrSensorData.totals[fsrPoint.leg] += weight;
  }
}
