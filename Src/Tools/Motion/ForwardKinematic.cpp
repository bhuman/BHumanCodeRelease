/**
 * @file ForwardKinematic.cpp
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 * @author Yannik Meinken
 */

#include "ForwardKinematic.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Math/BHMath.h"
#include "Math/Pose3f.h"
#include "Math/SE3fWithCov.h"
#include "Math/Rotation.h"

void ForwardKinematic::calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions,
                                         ENUM_INDEXED_ARRAY(SE3WithCov, Limbs::Limb)& limbs)
{
  const int sign = arm == Arms::left ? 1 : -1;
  Limbs::Limb shoulderLimb = arm == Arms::left ? Limbs::shoulderLeft : Limbs::shoulderRight;
  Joints::Joint shoulderJoint = arm == Arms::left ? Joints::lShoulderPitch : Joints::rShoulderPitch;

  SE3WithCov& shoulder = limbs[shoulderLimb];
  SE3WithCov& biceps = limbs[shoulderLimb + 1];
  SE3WithCov& elbow = limbs[shoulderLimb + 2];
  SE3WithCov& foreArm = limbs[shoulderLimb + 3];
  SE3WithCov& wrist = limbs[shoulderLimb + 4];

  shoulder = SE3WithCov(Pose3f(robotDimensions.armOffset.x(), robotDimensions.armOffset.y() * sign, robotDimensions.armOffset.z())) *= SE3WithCov(
      RotationMatrix::aroundY(joints.angles[shoulderJoint]), Vector3f(0.f, joints.variance[shoulderJoint], 0.f).asDiagonal());
  biceps = shoulder *
           SE3WithCov(RotationMatrix::aroundZ(joints.angles[shoulderJoint + 1]), Vector3f(0.f, 0.f, joints.variance[shoulderJoint + 1]).asDiagonal());
  elbow = (biceps + Vector3f(robotDimensions.upperArmLength, robotDimensions.yOffsetElbowToShoulder * sign, 0)) *= SE3WithCov(RotationMatrix::aroundX(
      joints.angles[shoulderJoint + 2]), Vector3f(joints.variance[shoulderJoint + 2], 0.f, 0.f).asDiagonal());
  foreArm = elbow * SE3WithCov(RotationMatrix::aroundZ(joints.angles[shoulderJoint + 3]), Vector3f(0.f, 0.f, joints.variance[shoulderJoint + 3]).asDiagonal());
  wrist = (foreArm + Vector3f(robotDimensions.xOffsetElbowToWrist, 0, 0)) *= SE3WithCov(RotationMatrix::aroundX(joints.angles[shoulderJoint + 4]),
                                                                                        Vector3f(joints.variance[shoulderJoint + 4], 0.f, 0.f).asDiagonal());
}

void ForwardKinematic::calculateLegChain(Legs::Leg leg, const JointAngles& joints, const RobotDimensions& robotDimensions,
                                         ENUM_INDEXED_ARRAY(SE3WithCov, Limbs::Limb)& limbs)
{
  const int sign = leg == Legs::left ? 1 : -1;
  Limbs::Limb pelvisLimb = leg == Legs::left ? Limbs::pelvisLeft : Limbs::pelvisRight;
  Joints::Joint hipJoint = leg == Legs::left ? Joints::lHipYawPitch : Joints::rHipYawPitch;

  SE3WithCov& pelvis = limbs[pelvisLimb];
  SE3WithCov& hip = limbs[pelvisLimb + 1];
  SE3WithCov& thigh = limbs[pelvisLimb + 2];
  SE3WithCov& tibia = limbs[pelvisLimb + 3];
  SE3WithCov& ankle = limbs[pelvisLimb + 4];
  SE3WithCov& foot = limbs[pelvisLimb + 5];

  pelvis = SE3WithCov(Pose3f(0.f, robotDimensions.yHipOffset * sign, 0.f)) *
           (Rotation::aroundX(pi_4 * sign) *
            SE3WithCov(Rotation::aroundZ(joints.angles[hipJoint] * -sign), Vector3f(0.f, 0.f, joints.variance[hipJoint]).asDiagonal()) *
            Rotation::aroundX(-pi_4 * sign)); // HitYawPitch
  hip = pelvis * SE3WithCov(RotationMatrix::aroundX(joints.angles[hipJoint + 1]), Vector3f(joints.variance[hipJoint + 1], 0.f, 0.f).asDiagonal()); // HipRoll
  thigh = hip * SE3WithCov(RotationMatrix::aroundY(joints.angles[hipJoint + 2]), Vector3f(0.f, joints.variance[hipJoint + 2], 0.f).asDiagonal()); // Hit Pitch
  tibia = (thigh + Vector3f(0, 0, -robotDimensions.upperLegLength)) *= SE3WithCov(RotationMatrix::aroundY(joints.angles[hipJoint + 3]),
                                                                                  Vector3f(0.f, joints.variance[hipJoint + 3], 0.f).asDiagonal()); // KneePitch
  ankle = (tibia + Vector3f(0, 0, -robotDimensions.lowerLegLength)) *= SE3WithCov(RotationMatrix::aroundY(joints.angles[hipJoint + 4]),
                                                                                  Vector3f(0.f, joints.variance[hipJoint + 4], 0.f).asDiagonal()); // AnklePitch
  foot = ankle * SE3WithCov(RotationMatrix::aroundX(joints.angles[hipJoint + 5]), Vector3f(joints.variance[hipJoint + 5], 0.f, 0.f).asDiagonal());
}

void ForwardKinematic::calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(SE3WithCov, Limbs::Limb)& limbs)
{
  limbs[Limbs::neck] = SE3WithCov(Pose3f(0.f, 0.f, robotDimensions.hipToNeckLength)) *= SE3WithCov(RotationMatrix::aroundZ(joints.angles[Joints::headYaw]),
                                                                                                   Vector3f(0.f, 0.f,
                                                                                                            joints.variance[Joints::headYaw]).asDiagonal());
  limbs[Limbs::head] = limbs[Limbs::neck] * SE3WithCov(RotationMatrix::aroundY(joints.angles[Joints::headPitch]),
                                                       Vector3f(0.f, joints.variance[Joints::headPitch], 0.f).asDiagonal());
}
