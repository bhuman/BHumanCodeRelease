/**
 * @file ForwardKinematic.h
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/RobotParts/Arms.h"
#include "Tools/RobotParts/Limbs.h"
#include "Tools/Streams/EnumIndexedArray.h"

struct JointAngles;
struct Pose3f;
struct RobotDimensions;

namespace ForwardKinematic
{
  void calculateArmChain(Arms::Arm arm, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs);
  void calculateLegChain(Legs::Leg leg, const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs);
  void calculateHeadChain(const JointAngles& joints, const RobotDimensions& robotDimensions, ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb)& limbs);
};
