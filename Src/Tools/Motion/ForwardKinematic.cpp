/**
 * @file ForwardKinematic.cpp
 * @author Alexander Härtl
 * @author Alexander Fabisch
 * @author Yannik Meinken
 */

#include "ForwardKinematic.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Math/BHMath.h"
#include "Math/Pose3f.h"
#include "Math/Rotation.h"
#include "Framework/Settings.h"
#include "Streaming/Global.h"

void ForwardKinematic::calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions,
                                         ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs)
{
  const int sign = arm == Arms::left ? 1 : -1;
  Limbs::Limb shoulderLimb = arm == Arms::left ? Limbs::shoulderLeft : Limbs::shoulderRight;
  Joints::Joint shoulderJoint = arm == Arms::left ? Joints::lShoulderPitch : Joints::rShoulderPitch;

  Pose3f& shoulder = limbs[shoulderLimb];
  Pose3f& biceps = limbs[shoulderLimb + 1];
  Pose3f& elbow = limbs[shoulderLimb + 2];
  Pose3f& foreArm = limbs[shoulderLimb + 3];
  Pose3f& wrist = limbs[shoulderLimb + 4];

  shoulder = Pose3f(robotDimensions.armOffset.x(), robotDimensions.armOffset.y() * sign, robotDimensions.armOffset.z()) *= RotationMatrix::aroundY(joints.angles[shoulderJoint]);
  biceps = (shoulder + Vector3f(robotDimensions.bicepsOffset.x(), robotDimensions.bicepsOffset.y() * sign, robotDimensions.bicepsOffset.z())) *= RotationMatrix::aroundX(joints.angles[shoulderJoint + 1]);
  elbow = (biceps + Vector3f(robotDimensions.yOffsetElbowToShoulder, robotDimensions.upperArmLength * sign, 0)) *= RotationMatrix::aroundY(joints.angles[shoulderJoint + 2]);
  foreArm = elbow * RotationMatrix::aroundZ(joints.angles[shoulderJoint + 3]);
  wrist = foreArm + Vector3f(0.f, robotDimensions.lowerArmLength * sign, 0.f);
}

void ForwardKinematic::calculateLegChain(Legs::Leg leg, const JointAngles& joints, const RobotDimensions& robotDimensions,
                                         ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs)
{
  const int sign = leg == Legs::left ? 1 : -1;
  Limbs::Limb pelvisLimb = leg == Legs::left ? Limbs::pelvisLeft : Limbs::pelvisRight;
  Joints::Joint hipJoint = leg == Legs::left ? Joints::lHipPitch : Joints::rHipPitch;

  Pose3f& waist = limbs[Limbs::waist];
  Pose3f& pelvis = limbs[pelvisLimb];
  Pose3f& hip = limbs[pelvisLimb + 1];
  Pose3f& thigh = limbs[pelvisLimb + 2];
  Pose3f& tibia = limbs[pelvisLimb + 3];
  Pose3f& ankle = limbs[pelvisLimb + 4];
  Pose3f& foot = limbs[pelvisLimb + 5];

  waist = Pose3f(Rotation::aroundZ(joints.angles[Joints::waistYaw]));
  pelvis = waist * Pose3f(0.f, robotDimensions.yHipOffset * sign, 0.f) * Rotation::aroundY(joints.angles[hipJoint]); // HipPitch
  hip = (pelvis + robotDimensions.hipPitchToRollOffset) *= RotationMatrix::aroundX(joints.angles[hipJoint + 1]); // HipRoll
  thigh = hip * RotationMatrix::aroundZ(joints.angles[hipJoint + 2]); // HipYaw
  tibia = (thigh + Vector3f(robotDimensions.xOffsetHipToKnee, 0, -robotDimensions.upperLegLength)) *= RotationMatrix::aroundY(joints.angles[hipJoint + 3]); // KneePitch
  ankle = (tibia + Vector3f(0, 0, -robotDimensions.lowerLegLength)) *= RotationMatrix::aroundY(joints.angles[hipJoint + 4]); // AnklePitch
  foot = (ankle + Vector3f(0, 0, robotDimensions.zOffsetAnklePitchToRoll)) *= RotationMatrix::aroundX(joints.angles[hipJoint + 5]);
}

void ForwardKinematic::calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs)
{
  limbs[Limbs::neck] = Pose3f(robotDimensions.hipToNeckOffset) *= RotationMatrix::aroundZ(joints.angles[Joints::headYaw]);
  limbs[Limbs::head] = limbs[Limbs::neck] * RotationMatrix::aroundY(joints.angles[Joints::headPitch]);
}
