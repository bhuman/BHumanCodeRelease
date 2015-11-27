/**
 * @file ForwardKinematic.h
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Arms.h"
#include "Tools/Limbs.h"

struct JointAngles;
struct Pose3f;
struct RobotDimensions;

namespace ForwardKinematic
{
  void calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions, Pose3f limbs[Limbs::numOfLimbs]);
  void calculateLegChain(bool left, const JointAngles& joints, const RobotDimensions& robotDimensions, Pose3f limbs[Limbs::numOfLimbs]);
  void calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, Pose3f limbs[Limbs::numOfLimbs]);
};
