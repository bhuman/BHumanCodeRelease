#pragma once

#include "RobotParts/Arms.h"
#include "RobotParts/Legs.h"
#include "Streaming/Enum.h"

namespace Joints
{
  ENUM(Joint,
  {,
    headYaw,
    headPitch,

    firstArmJoint,
    firstLeftArmJoint = firstArmJoint,

    lShoulderPitch = firstLeftArmJoint,
    lShoulderRoll,
    lElbowPitch,
    lElbowYaw,

    firstRightArmJoint,

    rShoulderPitch = firstRightArmJoint,
    rShoulderRoll,
    rElbowPitch,
    rElbowYaw,

    firstNoneArmJoint,
    waistYaw = firstNoneArmJoint, //< only present on the Booster T1

    firstLegJoint,
    firstLeftLegJoint = firstLegJoint,

    lHipPitch = firstLeftLegJoint,
    lHipRoll,
    lHipYaw,
    lKneePitch,
    lAnklePitch,
    lAnkleRoll,

    firstRightLegJoint,

    rHipPitch = firstRightLegJoint,
    rHipRoll,
    rHipYaw,
    rKneePitch,
    rAnklePitch,
    rAnkleRoll,
  });

  ENUM(ArmJoint,
  {,
    shoulderPitch,
    shoulderRoll,
    elbowPitch,
    elbowYaw,
  });

  static_assert(shoulderPitch == lShoulderPitch - firstLeftArmJoint && shoulderPitch == rShoulderPitch - firstRightArmJoint);
  static_assert(shoulderRoll == lShoulderRoll - firstLeftArmJoint && shoulderRoll == rShoulderRoll - firstRightArmJoint);
  static_assert(elbowPitch == lElbowPitch - firstLeftArmJoint && elbowPitch == rElbowPitch - firstRightArmJoint);
  static_assert(elbowYaw == lElbowYaw - firstLeftArmJoint && elbowYaw == rElbowYaw - firstRightArmJoint);

  ENUM(LegJoint,
  {,
    hipPitch,
    hipRoll,
    hipYaw,
    kneePitch,
    anklePitch,
    ankleRoll,
  });

  static_assert(hipPitch == lHipPitch - firstLeftLegJoint && hipPitch == rHipPitch - firstRightLegJoint);
  static_assert(hipRoll == lHipRoll - firstLeftLegJoint && hipRoll == rHipRoll - firstRightLegJoint);
  static_assert(hipYaw == lHipYaw - firstLeftLegJoint && hipYaw == rHipYaw - firstRightLegJoint);
  static_assert(kneePitch == lKneePitch - firstLeftLegJoint && kneePitch == rKneePitch - firstRightLegJoint);
  static_assert(anklePitch == lAnklePitch - firstLeftLegJoint && anklePitch == rAnklePitch - firstRightLegJoint);
  static_assert(ankleRoll == lAnkleRoll - firstLeftLegJoint && ankleRoll == rAnkleRoll - firstRightLegJoint);

  inline Joint combine(const Arms::Arm arm, const ArmJoint joint)
  {
    static const Joint firstArmJoints[2] = {firstLeftArmJoint, firstRightArmJoint};
    return static_cast<Joint>(firstArmJoints[arm] + static_cast<Joint>(joint));
  }

  inline Joint combine(const Legs::Leg leg, const LegJoint joint)
  {
    static const Joint firstLegJoints[2] = {firstLeftLegJoint, firstRightLegJoint};
    return static_cast<Joint>(firstLegJoints[leg] + static_cast<Joint>(joint));
  }

  inline Joint mirror(const Joint joint)
  {
    switch(joint)
    {
      case Joints::lShoulderPitch:
      case Joints::lShoulderRoll:
      case Joints::lElbowPitch:
      case Joints::lElbowYaw:
        return Joint(joint - Joints::firstLeftArmJoint + Joints::firstRightArmJoint);
      case Joints::rShoulderPitch:
      case Joints::rShoulderRoll:
      case Joints::rElbowPitch:
      case Joints::rElbowYaw:
        return Joint(joint - Joints::firstRightArmJoint + Joints::firstLeftArmJoint);
      case Joints::lHipPitch:
      case Joints::lHipRoll:
      case Joints::lHipYaw:
      case Joints::lKneePitch:
      case Joints::lAnklePitch:
      case Joints::lAnkleRoll:
        return Joint(joint - Joints::firstLeftLegJoint + Joints::firstRightLegJoint);
      case Joints::rHipPitch:
      case Joints::rHipRoll:
      case Joints::rHipYaw:
      case Joints::rKneePitch:
      case Joints::rAnklePitch:
      case Joints::rAnkleRoll:
        return Joint(joint - Joints::firstRightLegJoint + Joints::firstLeftLegJoint);
      default:
        return joint;
    }
  }

  inline bool canNegate(const Joint joint)
  {
    switch(joint)
    {
      case Joints::headYaw:
      case Joints::lShoulderRoll:
      case Joints::lElbowYaw:
      case Joints::rShoulderRoll:
      case Joints::rElbowYaw:
      case Joints::waistYaw:
      case Joints::lHipRoll:
      case Joints::lHipYaw:
      case Joints::lAnkleRoll:
      case Joints::rHipRoll:
      case Joints::rHipYaw:
      case Joints::rAnkleRoll:
        return true;
      default:
        return false;
    }
  }
}
