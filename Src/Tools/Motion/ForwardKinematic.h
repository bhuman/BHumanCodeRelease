/**
 * @file ForwardKinematic.h
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Streaming/EnumIndexedArray.h"
#include "RobotParts/Arms.h"
#include "RobotParts/Limbs.h"

struct JointAngles;
struct SE3WithCov;
struct RobotDimensions;

namespace ForwardKinematic
{
  void calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(SE3WithCov, Limbs::Limb)& limbs);

  void calculateLegChain(Legs::Leg leg, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(SE3WithCov, Limbs::Limb)& limbs);

  void calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(SE3WithCov, Limbs::Limb)& limbs);
};
