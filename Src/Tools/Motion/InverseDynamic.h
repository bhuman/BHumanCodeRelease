/**
 * @file InverseDynamic.h
 *
 * This file declares a class to calculate inverse dynamics using the recursive Newton-Euler algorithm (RNEA).
 *
 * @author Felix Wenk
 * @author Arne Hasselbring
 */

#pragma once

#include "Framework/Settings.h"
#include "Math/Eigen.h"
#include "Math/Pose3f.h"
#include "RobotParts/Joints.h"
#include "RobotParts/Limbs.h"
#include "Tools/Math/SpatialVector.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Sensing/JointDynamics.h"

class InverseDynamic
{
public:
  /**
   * Calculates the torques on all joints for given angles, velocties and accelerations, assuming only gravity as external force.
   * @param jointDynamics \c angles, \c velocities and \c accelerations must be filled, \c torques are filled by this function.
   * @param gravityInTorso The direction and length of the gravity (in mm/s^2)
   */
  static void calculateJointTorques(JointDynamics& jointDynamics, const Vector3f& gravityInTorso, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration);

private:
  /** A single node in the kinematic tree. */
  struct Node
  {
    Joints::Joint joint; /**< The joint that links parent to child. */
    Limbs::Limb parent;
    Limbs::Limb child;
  };

  /** Helper struct for intermediate results of the RNEA. */
  struct Body
  {
    Pose3f bodyInParent; /**< Pose of the body frame relative to its parent body's frame. */
    Pose3f parentInBody; /**< Pose of the parent body's frame relative to its own frame. */
    SpatialVector3f<true> mode; /**< The spatial vector along which the joint between this body and its parent moves. */

    SpatialVector3f<true> v; /**< The spatial velocity of this body (in rad/s, mm/s). */
    SpatialVector3f<true> a; /**< The spatial acceleration of this body (in rad/s^2, mm/s^2). */
    SpatialVector3f<false> f; /**< The spatial force exerted on this body (in g*mm^2*rad/s^2 (a.k.a. uNmm), g*mm/s^2 (a.k.a. uN)). */
  };

  /** Calculates the poses of the head limbs in \c bodies. */
  static void calculateHeadPoses(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const RobotDimensions& robotDimensions);

  /** Calculates the poses of one arm's limbs in \c bodies. */
  static void calculateArmPoses(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const RobotDimensions& robotDimensions, bool left);

  /** Calculates the poses of one leg's limbs in \c bodies. */
  static void calculateLegPoses(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const RobotDimensions& robotDimensions, bool left);

  /** Performs a forward step of the RNEA on one joint (i.e. calculates spatial velocity and acceleration and the resulting force). */
  static void forward(std::array<Body, Limbs::numOfLimbs>& bodies, const JointDynamics& jointDynamics, const MassCalibration& massCalibration, const Node& node);

  /** Performs a backward step of the RNEA on one joint (i.e. propagates forces back to root and accumulates torques). */
  static void backward(std::array<Body, Limbs::numOfLimbs>& bodies, JointDynamics& jointDynamics, const Node& node);

  /** Generate kinematic tree for all supported robots. */
  static std::array<std::vector<Node>, Settings::numOfRobotTypes> generateKinematicTree();

  static std::array<std::vector<Node>, Settings::numOfRobotTypes> kinematicTree; /**< Kinematic trees for all supported robots. */
};
