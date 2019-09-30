/**
 * @file MotionUtilities.h
 * provides functions for interacting with motions
 * @author Bernd Poppinga
 */

#pragma once

#include "Tools/Math/Pose3f.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"

struct JointAngles;
struct Pose3f;

namespace MotionUtilities
{
  /**
   * The method copies all joint angles from one joint request to another,
   * but only those that should not be ignored.
   * @param source The source joint request. All angles != JointAngles::ignore will be copied.
   * @param target The target joint request.
   */
  void copy(const JointRequest& source, JointRequest& target,
            const StiffnessSettings& theStiffnessSettings,
            const Joints::Joint startJoint, const Joints::Joint endJoint);

  /**
   * The method interpolates between two joint requests.
   * @param from The first source joint request. This one is the starting point.
   * @param to The second source joint request. This one has to be reached over time.
   * @param fromRatio The ratio of "from" in the target joint request.
   * @param target The target joint request.
   * @param interpolateStiffness Whether to interpolate stiffness.
   */
  void interpolate(const JointRequest& from, const JointRequest& to, float fromRatio, JointRequest& target, bool interpolateStiffness,
                   const StiffnessSettings& theStiffnessSettings, const JointAngles& lastJointAngles,
                   const Joints::Joint startJoint, const Joints::Joint endJoint);

  /**
   * The method interpolates between two joint requests.
   * @param joints destination
   * @param alpha the ratio which determines how fast to interpolate
   * @param threshold when a certain similarity is reached return true
   * @param theJointRequest the joint to interpolate from
   * @param theJointAngles
   */
  bool interpolate(JointRequest& joints, const float alpha, const float threshold, const JointRequest& theJointRequest, const JointAngles& theJointAngles,
                   const JointRequest& theStandLegRequest);

  void interpolate(const JointAngles& from, const JointRequest& to, float& ratio, JointRequest& target, const JointAngles& theJointAngles);
  void stand(JointRequest& jointRequest);
  void sit(JointRequest& jointRequest);
};
