/**
 * @file ForwardKinematic.h
 * @author Alexander Härtl
 * @author Alexander Fabisch
 */

#pragma once

#include "Streaming/EnumIndexedArray.h"
#include "RobotParts/Arms.h"
#include "RobotParts/Limbs.h"

struct JointAngles;
struct Pose3f;
struct RobotDimensions;

namespace ForwardKinematic
{
  void calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs);

  void calculateLegChain(Legs::Leg leg, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs);

  void calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs);
};
