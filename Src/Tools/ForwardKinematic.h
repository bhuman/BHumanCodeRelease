/**
 * @file ForwardKinematic.h
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Math/Pose3D.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"

class ForwardKinematic
{
public:
  static void calculateArmChain(bool left, const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration, Pose3D limbs[MassCalibration::numOfLimbs])
  {
    int sign = left ? -1 : 1;
    MassCalibration::Limb shoulder = left ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;
    JointData::Joint arm0 = left ? JointData::LShoulderPitch : JointData::RShoulderPitch;

    limbs[shoulder + 0] = Pose3D(robotDimensions.armOffset.x, robotDimensions.armOffset.y * -sign, robotDimensions.armOffset.z)
                          .rotateY(-joints.angles[arm0 + 0]);
    limbs[shoulder + 1] = Pose3D(limbs[shoulder + 0])
                          .rotateZ(joints.angles[arm0 + 1] * -sign);
    limbs[shoulder + 2] = Pose3D(limbs[shoulder + 1])
                          .translate(robotDimensions.upperArmLength, robotDimensions.yElbowShoulder * -sign, 0)
                          .rotateX(joints.angles[arm0 + 2] * -sign);
    limbs[shoulder + 3] = Pose3D(limbs[shoulder + 2])
                          .rotateZ(joints.angles[arm0 + 3] * -sign);
  }

  static void calculateLegChain(bool left, const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration, Pose3D limbs[MassCalibration::numOfLimbs])
  {
    int sign = left ? -1 : 1;
    MassCalibration::Limb pelvis = left ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
    JointData::Joint leg0 = left ? JointData::LHipYawPitch : JointData::RHipYawPitch;

    limbs[pelvis + 0] =  Pose3D(0, robotDimensions.lengthBetweenLegs / 2.0f * -sign, 0)
                         .rotateX(-pi_4 * sign)
                         .rotateZ(joints.angles[leg0 + 0] * sign)
                         .rotateX(pi_4 * sign);
    limbs[pelvis + 1] = Pose3D(limbs[pelvis + 0])
                        .rotateX(joints.angles[leg0 + 1] * sign);
    limbs[pelvis + 2] = Pose3D(limbs[pelvis + 1])
                        .rotateY(joints.angles[leg0 + 2]);
    limbs[pelvis + 3] = Pose3D(limbs[pelvis + 2])
                        .translate(0, 0, -robotDimensions.upperLegLength)
                        .rotateY(joints.angles[leg0 + 3]);
    limbs[pelvis + 4] = Pose3D(limbs[pelvis + 3])
                        .translate(0, 0, -robotDimensions.lowerLegLength)
                        .rotateY(joints.angles[leg0 + 4]);
    limbs[pelvis + 5] = Pose3D(limbs[pelvis + 4])
                        .rotateX(joints.angles[leg0 + 5] * sign);
  }

  static void calculateHeadChain(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration, Pose3D limbs[MassCalibration::numOfLimbs])
  {
    limbs[MassCalibration::neck] = Pose3D(0, 0, robotDimensions.zLegJoint1ToHeadPan)
                                   .rotateZ(joints.angles[JointData::HeadYaw]);
    limbs[MassCalibration::head] = Pose3D(limbs[MassCalibration::neck])
                                   .rotateY(-joints.angles[JointData::HeadPitch]);
  }
};
