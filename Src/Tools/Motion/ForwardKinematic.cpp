/**
 * @file ForwardKinematic.cpp
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#include "ForwardKinematic.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Rotation.h"

void ForwardKinematic::calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs)
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
  biceps = shoulder * RotationMatrix::aroundZ(joints.angles[shoulderJoint + 1]);
  elbow = (biceps + Vector3f(robotDimensions.upperArmLength, robotDimensions.yOffsetElbowToShoulder * sign, 0)) *= RotationMatrix::aroundX(joints.angles[shoulderJoint + 2]);
  foreArm = elbow * RotationMatrix::aroundZ(joints.angles[shoulderJoint + 3]);
  wrist = (foreArm + Vector3f(robotDimensions.xOffsetElbowToWrist, 0, 0)) *= RotationMatrix::aroundX(joints.angles[shoulderJoint + 4]);
}

void ForwardKinematic::calculateLegChain(Legs::Leg leg, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs)
{
  const int sign = leg == Legs::left ? 1 : -1;
  Limbs::Limb pelvisLimb = leg == Legs::left ? Limbs::pelvisLeft : Limbs::pelvisRight;
  Joints::Joint hipJoint = leg == Legs::left ? Joints::lHipYawPitch : Joints::rHipYawPitch;

  Pose3f& pelvis = limbs[pelvisLimb];
  Pose3f& hip = limbs[pelvisLimb + 1];
  Pose3f& thigh = limbs[pelvisLimb + 2];
  Pose3f& tibia = limbs[pelvisLimb + 3];
  Pose3f& ankle = limbs[pelvisLimb + 4];
  Pose3f& foot = limbs[pelvisLimb + 5];

  pelvis = Pose3f(0.f, robotDimensions.yHipOffset * sign, 0.f) * (Rotation::aroundX(pi_4 * sign) * Rotation::aroundZ(joints.angles[hipJoint] * -sign) * Rotation::aroundX(-pi_4 * sign));
  hip = pelvis * RotationMatrix::aroundX(joints.angles[hipJoint + 1]);
  thigh = hip * RotationMatrix::aroundY(joints.angles[hipJoint + 2]);
  tibia = (thigh + Vector3f(0, 0, -robotDimensions.upperLegLength)) *= RotationMatrix::aroundY(joints.angles[hipJoint + 3]);
  ankle = (tibia + Vector3f(0, 0, -robotDimensions.lowerLegLength)) *= RotationMatrix::aroundY(joints.angles[hipJoint + 4]);
  foot = ankle * RotationMatrix::aroundX(joints.angles[hipJoint + 5]);
}

void ForwardKinematic::calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs)
{
  limbs[Limbs::neck] = Pose3f(0.f, 0.f, robotDimensions.hipToNeckLength) *= RotationMatrix::aroundZ(joints.angles[Joints::headYaw]);
  limbs[Limbs::head] = limbs[Limbs::neck] * RotationMatrix::aroundY(joints.angles[Joints::headPitch]);
}
