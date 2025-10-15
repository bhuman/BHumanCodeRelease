/**
 * @file InverseDynamic.cpp
 *
 * This file implements a class to calculate inverse dynamics using the recursive Newton-Euler algorithm (RNEA).
 *
 * @author Felix Wenk
 * @author Arne Hasselbring
 */

#include "InverseDynamic.h"
#include "Math/Constants.h"
#include "Streaming/Global.h"
#include <ranges>

std::array<std::vector<InverseDynamic::Node>, Settings::numOfRobotTypes> InverseDynamic::generateKinematicTree()
{
  std::array<std::vector<Node>, Settings::numOfRobotTypes> tree;
  tree[Settings::nao] =
  {
    {Joints::headYaw, Limbs::torso, Limbs::neck},
    {Joints::headPitch, Limbs::neck, Limbs::head},
    {Joints::lShoulderPitch, Limbs::torso, Limbs::shoulderLeft},
    {Joints::lShoulderRoll, Limbs::shoulderLeft, Limbs::bicepsLeft},
    {Joints::lElbowYaw, Limbs::bicepsLeft, Limbs::elbowLeft},
    {Joints::lElbowRoll, Limbs::elbowLeft, Limbs::foreArmLeft},
    {Joints::lWristYaw, Limbs::foreArmLeft, Limbs::wristLeft},
    {Joints::rShoulderPitch, Limbs::torso, Limbs::shoulderRight},
    {Joints::rShoulderRoll, Limbs::shoulderRight, Limbs::bicepsRight},
    {Joints::rElbowYaw, Limbs::bicepsRight, Limbs::elbowRight},
    {Joints::rElbowRoll, Limbs::elbowRight, Limbs::foreArmRight},
    {Joints::rWristYaw, Limbs::foreArmRight, Limbs::wristRight},
    {Joints::lHipYawPitch, Limbs::torso, Limbs::pelvisLeft},
    {Joints::lHipRoll, Limbs::pelvisLeft, Limbs::hipLeft},
    {Joints::lHipPitch, Limbs::hipLeft, Limbs::thighLeft},
    {Joints::lKneePitch, Limbs::thighLeft, Limbs::tibiaLeft},
    {Joints::lAnklePitch, Limbs::tibiaLeft, Limbs::ankleLeft},
    {Joints::lAnkleRoll, Limbs::ankleLeft, Limbs::footLeft},
    {Joints::rHipYawPitch, Limbs::torso, Limbs::pelvisRight},
    {Joints::rHipRoll, Limbs::pelvisRight, Limbs::hipRight},
    {Joints::rHipPitch, Limbs::hipRight, Limbs::thighRight},
    {Joints::rKneePitch, Limbs::thighRight, Limbs::tibiaRight},
    {Joints::rAnklePitch, Limbs::tibiaRight, Limbs::ankleRight},
    {Joints::rAnkleRoll, Limbs::ankleRight, Limbs::footRight},
  };
  tree[Settings::t1] =
  {
    {Joints::headYaw, Limbs::torso, Limbs::neck},
    {Joints::headPitch, Limbs::neck, Limbs::head},
    {Joints::lShoulderPitch, Limbs::torso, Limbs::shoulderLeft},
    {Joints::lShoulderRoll, Limbs::shoulderLeft, Limbs::bicepsLeft},
    {Joints::lElbowYaw, Limbs::bicepsLeft, Limbs::elbowLeft},
    {Joints::lElbowRoll, Limbs::elbowLeft, Limbs::foreArmLeft},
    {Joints::rShoulderPitch, Limbs::torso, Limbs::shoulderRight},
    {Joints::rShoulderRoll, Limbs::shoulderRight, Limbs::bicepsRight},
    {Joints::rElbowYaw, Limbs::bicepsRight, Limbs::elbowRight},
    {Joints::rElbowRoll, Limbs::elbowRight, Limbs::foreArmRight},
    {Joints::waistYaw, Limbs::torso, Limbs::waist},
    {Joints::lHipPitch, Limbs::waist, Limbs::pelvisLeft},
    {Joints::lHipRoll, Limbs::pelvisLeft, Limbs::hipLeft},
    {Joints::lHipYaw, Limbs::hipLeft, Limbs::thighLeft},
    {Joints::lKneePitch, Limbs::thighLeft, Limbs::tibiaLeft},
    {Joints::lAnklePitch, Limbs::tibiaLeft, Limbs::ankleLeft},
    {Joints::lAnkleRoll, Limbs::ankleLeft, Limbs::footLeft},
    {Joints::rHipPitch, Limbs::waist, Limbs::pelvisRight},
    {Joints::rHipRoll, Limbs::pelvisRight, Limbs::hipRight},
    {Joints::rHipYaw, Limbs::hipRight, Limbs::thighRight},
    {Joints::rKneePitch, Limbs::thighRight, Limbs::tibiaRight},
    {Joints::rAnklePitch, Limbs::tibiaRight, Limbs::ankleRight},
    {Joints::rAnkleRoll, Limbs::ankleRight, Limbs::footRight},
  };

  tree[Settings::k1] =
  {
    {Joints::headYaw, Limbs::torso, Limbs::neck},
    {Joints::headPitch, Limbs::neck, Limbs::head},
    {Joints::lShoulderPitch, Limbs::torso, Limbs::shoulderLeft},
    {Joints::lShoulderRoll, Limbs::shoulderLeft, Limbs::bicepsLeft},
    {Joints::lElbowYaw, Limbs::bicepsLeft, Limbs::elbowLeft},
    {Joints::lElbowRoll, Limbs::elbowLeft, Limbs::foreArmLeft},
    {Joints::rShoulderPitch, Limbs::torso, Limbs::shoulderRight},
    {Joints::rShoulderRoll, Limbs::shoulderRight, Limbs::bicepsRight},
    {Joints::rElbowYaw, Limbs::bicepsRight, Limbs::elbowRight},
    {Joints::rElbowRoll, Limbs::elbowRight, Limbs::foreArmRight},
    {Joints::lHipPitch, Limbs::waist, Limbs::pelvisLeft},
    {Joints::lHipRoll, Limbs::pelvisLeft, Limbs::hipLeft},
    {Joints::lHipYaw, Limbs::hipLeft, Limbs::thighLeft},
    {Joints::lKneePitch, Limbs::thighLeft, Limbs::tibiaLeft},
    {Joints::lAnklePitch, Limbs::tibiaLeft, Limbs::ankleLeft},
    {Joints::lAnkleRoll, Limbs::ankleLeft, Limbs::footLeft},
    {Joints::rHipPitch, Limbs::waist, Limbs::pelvisRight},
    {Joints::rHipRoll, Limbs::pelvisRight, Limbs::hipRight},
    {Joints::rHipYaw, Limbs::hipRight, Limbs::thighRight},
    {Joints::rKneePitch, Limbs::thighRight, Limbs::tibiaRight},
    {Joints::rAnklePitch, Limbs::tibiaRight, Limbs::ankleRight},
    {Joints::rAnkleRoll, Limbs::ankleRight, Limbs::footRight},
  };
  return tree;
}

std::array<std::vector<InverseDynamic::Node>, Settings::numOfRobotTypes> InverseDynamic::kinematicTree = generateKinematicTree();

void InverseDynamic::calculateJointTorques(JointDynamics& jointDynamics, const Vector3f& gravityInTorso, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration)
{
  std::array<Body, Limbs::numOfLimbs> bodies;

  calculateHeadPoses(bodies, jointDynamics, robotDimensions);
  calculateArmPoses(bodies, jointDynamics, robotDimensions, true);
  calculateArmPoses(bodies, jointDynamics, robotDimensions, false);
  calculateLegPoses(bodies, jointDynamics, robotDimensions, true);
  calculateLegPoses(bodies, jointDynamics, robotDimensions, false);

  for(Body& body : bodies)
    body.parentInBody = body.bodyInParent.inverse();

  bodies[Limbs::torso].v = SpatialVector3f<true>();
  bodies[Limbs::torso].a = SpatialVector3f<true>(Vector3f::Zero(), gravityInTorso);
  bodies[Limbs::torso].f = SpatialVector3f<true>::applyInertia(massCalibration.masses[Limbs::torso], bodies[Limbs::torso].a);

  for(const Node& node : kinematicTree[Global::getSettings().robotType])
    forward(bodies, jointDynamics, massCalibration, node);

  jointDynamics.torques.fill(0.f);
  for(const Node& node : kinematicTree[Global::getSettings().robotType] | std::views::reverse)
    backward(bodies, jointDynamics, node);
}

void InverseDynamic::calculateHeadPoses(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const RobotDimensions& robotDimensions)
{
  // neck in torso
  bodies[Limbs::neck].bodyInParent = Pose3f(RotationMatrix::aroundZ(jointDynamics.angles[Joints::headYaw]),
                                            robotDimensions.hipToNeckOffset);
  bodies[Limbs::neck].mode = SpatialVector3f<true>(Vector3f(0.0f, 0.0f, 1.0f), Vector3f::Zero());

  // head in neck
  bodies[Limbs::head].bodyInParent = Pose3f(RotationMatrix::aroundY(jointDynamics.angles[Joints::headPitch]),
                                            Vector3f::Zero());
  bodies[Limbs::head].mode = SpatialVector3f<true>(Vector3f(0.0f, 1.0f, 0.0f), Vector3f::Zero());
}

void InverseDynamic::calculateArmPoses(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const RobotDimensions& robotDimensions, bool left)
{
  const float sign = left ? 1.f : -1.f;
  const Limbs::Limb shoulder = left ? Limbs::shoulderLeft : Limbs::shoulderRight;
  const Joints::Joint arm0 = left ? Joints::lShoulderPitch : Joints::rShoulderPitch;

  // shoulder in torso
  bodies[shoulder].bodyInParent = Pose3f(RotationMatrix::aroundY(jointDynamics.angles[arm0]),
                                         Vector3f(robotDimensions.armOffset.x(), robotDimensions.armOffset.y() * sign, robotDimensions.armOffset.z()));
  bodies[shoulder].mode = SpatialVector3f<true>(Vector3f(0.f, 1.f, 0.f), Vector3f::Zero());

  switch(Global::getSettings().robotType)
  {
    case Settings::nao:
      // biceps in shoulder
      bodies[shoulder + 1].bodyInParent = Pose3f(RotationMatrix::aroundZ(jointDynamics.angles[arm0 + 1]),
                                                 Vector3f::Zero());
      bodies[shoulder + 1].mode = SpatialVector3f<true>(Vector3f(0.f, 0.f, 1.f), Vector3f::Zero());

      // elbow in biceps
      bodies[shoulder + 2].bodyInParent = Pose3f(RotationMatrix::aroundX(jointDynamics.angles[arm0 + 2]),
                                                 Vector3f(robotDimensions.upperArmLength, robotDimensions.yOffsetElbowToShoulder * sign, 0.f));
      bodies[shoulder + 2].mode = SpatialVector3f<true>(Vector3f(1.f, 0.f, 0.f), Vector3f::Zero());

      // foreArm in elbow
      bodies[shoulder + 3].bodyInParent = Pose3f(RotationMatrix::aroundZ(jointDynamics.angles[arm0 + 3]),
                                                 Vector3f::Zero());
      bodies[shoulder + 3].mode = SpatialVector3f<true>(Vector3f(0.f, 0.f, 1.f), Vector3f::Zero());

      // wrist in foreArm
      bodies[shoulder + 4].bodyInParent = Pose3f(RotationMatrix::aroundX(jointDynamics.angles[arm0 + 4]),
                                                 Vector3f(robotDimensions.xOffsetElbowToWrist, 0.f, 0.f));
      bodies[shoulder + 4].mode = SpatialVector3f<true>(Vector3f(1.f, 0.f, 0.f), Vector3f::Zero());
      break;
    case Settings::t1:
    case Settings::k1:
      // biceps in shoulder
      bodies[shoulder + 1].bodyInParent = Pose3f(RotationMatrix::aroundX(jointDynamics.angles[arm0 + 1]),
                                                 Vector3f::Zero());
      bodies[shoulder + 1].mode = SpatialVector3f<true>(Vector3f(1.f, 0.f, 0.f), Vector3f::Zero());

      // elbow in biceps
      bodies[shoulder + 2].bodyInParent = Pose3f(RotationMatrix::aroundY(jointDynamics.angles[arm0 + 2]),
                                                 Vector3f(0.f, robotDimensions.upperArmLength * sign, 0.f));
      bodies[shoulder + 2].mode = SpatialVector3f<true>(Vector3f(0.f, 1.f, 0.f), Vector3f::Zero());

      // foreArm in elbow
      bodies[shoulder + 3].bodyInParent = Pose3f(RotationMatrix::aroundZ(jointDynamics.angles[arm0 + 3]),
                                                 Vector3f::Zero());
      bodies[shoulder + 3].mode = SpatialVector3f<true>(Vector3f(0.f, 0.f, 1.f), Vector3f::Zero());

      // no wrist
      break;
  }
}

void InverseDynamic::calculateLegPoses(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const RobotDimensions& robotDimensions, bool left)
{
  const float sign = left ? 1.f : -1.f;
  const Limbs::Limb pelvis = left ? Limbs::pelvisLeft : Limbs::pelvisRight;
  const Joints::Joint leg0 = left ? Joints::lHipYawPitch : Joints::rHipYawPitch;

  switch(Global::getSettings().robotType)
  {
    case Settings::nao:
    {
      // pelvis in torso
      bodies[pelvis + 0].bodyInParent = Pose3f(RotationMatrix::aroundX(pi_4 * sign) * RotationMatrix::aroundZ(jointDynamics.angles[leg0] * -sign) * RotationMatrix::aroundX(pi_4 * -sign),
                                               Vector3f(0.f, robotDimensions.yHipOffset * sign, 0.f));
      bodies[pelvis + 0].mode = SpatialVector3f<true>(Vector3f(0.f, std::sqrt(0.5f), -sign * std::sqrt(0.5f)), Vector3f::Zero());

      // hip in pelvis
      bodies[pelvis + 1].bodyInParent = Pose3f(RotationMatrix::aroundX(jointDynamics.angles[leg0 + 1]),
                                               Vector3f::Zero());
      bodies[pelvis + 1].mode = SpatialVector3f<true>(Vector3f(1.f, 0.f, 0.f), Vector3f::Zero());

      // thigh in hip
      bodies[pelvis + 2].bodyInParent = Pose3f(RotationMatrix::aroundY(jointDynamics.angles[leg0 + 2]),
                                               Vector3f::Zero());
      bodies[pelvis + 2].mode = SpatialVector3f<true>(Vector3f(0.f, 1.f, 0.f), Vector3f::Zero());
      break;
    }
    case Settings::t1:
    case Settings::k1:
      if(left)
      {
        // waist in torso
        bodies[Limbs::waist].bodyInParent = Pose3f(RotationMatrix::aroundZ(jointDynamics.angles[Joints::waistYaw]),
                                                   Vector3f::Zero());
        bodies[Limbs::waist].mode = SpatialVector3f<true>(Vector3f(0.f, 0.f, 1.f), Vector3f::Zero());
      }

      // pelvis in waist
      bodies[pelvis + 0].bodyInParent = Pose3f(RotationMatrix::aroundY(jointDynamics.angles[leg0 + 2]),
                                               Vector3f(0.f, robotDimensions.yHipOffset * sign, 0.f));
      bodies[pelvis + 0].mode = SpatialVector3f<true>(Vector3f(0.f, 1.f, 0.f), Vector3f::Zero());

      // hip in pelvis
      bodies[pelvis + 1].bodyInParent = Pose3f(RotationMatrix::aroundX(jointDynamics.angles[leg0 + 1]),
                                               robotDimensions.hipPitchToRollOffset);
      bodies[pelvis + 1].mode = SpatialVector3f<true>(Vector3f(1.f, 0.f, 0.f), Vector3f::Zero());

      // thigh in hip
      bodies[pelvis + 2].bodyInParent = Pose3f(RotationMatrix::aroundZ(jointDynamics.angles[leg0]),
                                               Vector3f::Zero());
      bodies[pelvis + 2].mode = SpatialVector3f<true>(Vector3f(0.f, 0.f, 1.f), Vector3f::Zero());
      break;
  }

  // tibia in thigh
  bodies[pelvis + 3].bodyInParent = Pose3f(RotationMatrix::aroundY(jointDynamics.angles[leg0 + 3]),
                                           Vector3f(robotDimensions.xOffsetHipToKnee, 0.f, -robotDimensions.upperLegLength));
  bodies[pelvis + 3].mode = SpatialVector3f<true>(Vector3f(0.f, 1.f, 0.f), Vector3f::Zero());

  // ankle in tibia
  bodies[pelvis + 4].bodyInParent = Pose3f(RotationMatrix::aroundY(jointDynamics.angles[leg0 + 4]),
                                           Vector3f(0.f, 0.f, -robotDimensions.lowerLegLength));
  bodies[pelvis + 4].mode = SpatialVector3f<true>(Vector3f(0.f, 1.f, 0.f), Vector3f::Zero());

  // foot in ankle
  bodies[pelvis + 5].bodyInParent = Pose3f(RotationMatrix::aroundX(jointDynamics.angles[leg0 + 5]),
                                           Vector3f(0.f, 0.f, robotDimensions.zOffsetAnklePitchToRoll));
  bodies[pelvis + 5].mode = SpatialVector3f<true>(Vector3f(1.f, 0.f, 0.f), Vector3f::Zero());
}

void InverseDynamic::forward(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const MassCalibration& massCalibration, const Node& node)
{
  const SpatialVector3f<true> vj = bodies[node.child].mode * jointDynamics.velocities[node.joint];
  bodies[node.child].v = SpatialVector3f<true>::applyPose3f(bodies[node.child].parentInBody, bodies[node.parent].v) + vj;
  bodies[node.child].a = SpatialVector3f<true>::applyPose3f(bodies[node.child].parentInBody, bodies[node.parent].a) + bodies[node.child].mode * jointDynamics.accelerations[node.joint] + bodies[node.child].v.cross(vj);
  bodies[node.child].f = SpatialVector3f<true>::applyInertia(massCalibration.masses[node.child], bodies[node.child].a) + bodies[node.child].v.cross(SpatialVector3f<true>::applyInertia(massCalibration.masses[node.child], bodies[node.child].v));
}

void InverseDynamic::backward(std::array<Body, Limbs::numOfLimbs>& bodies, JointDynamics& jointDynamics, const Node& node)
{
  jointDynamics.torques[node.joint] += bodies[node.child].mode.dot(bodies[node.child].f);
  bodies[node.parent].f += SpatialVector3f<false>::applyPose3f(bodies[node.child].bodyInParent, bodies[node.child].f);
}
