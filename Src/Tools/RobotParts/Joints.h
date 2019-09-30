#pragma once

#include "Tools/Streams/Enum.h"
#include "Arms.h"
#include "Legs.h"
#include <vector>

namespace Joints
{
  ENUM(Joint,
  {, //ALL ENUMS HERE MUST BE IN THE SAME ORDER LIKE THE ENUMS BELOW
    headYaw,
    headPitch,

    firstArmJoint,
    firstLeftArmJoint = firstArmJoint,

    lShoulderPitch = firstLeftArmJoint,
    lShoulderRoll,
    lElbowYaw,
    lElbowRoll,
    lWristYaw,
    lHand, //< not an Angle, instead %

    firstRightArmJoint,

    rShoulderPitch = firstRightArmJoint,
    rShoulderRoll,
    rElbowYaw,
    rElbowRoll,
    rWristYaw,
    rHand, //< not an Angle, instead %

    firstLegJoint,
    firstLeftLegJoint = firstLegJoint,

    lHipYawPitch = firstLeftLegJoint,
    lHipRoll,
    lHipPitch,
    lKneePitch,
    lAnklePitch,
    lAnkleRoll,

    firstRightLegJoint,

    rHipYawPitch = firstRightLegJoint, //< not a joint in the real NAO
    rHipRoll,
    rHipPitch,
    rKneePitch,
    rAnklePitch,
    rAnkleRoll,
  });

  using stdVectorJoint = std::vector<Joint>;

  ENUM(JointArmVarieties,
  {,
    shoulderPitch,
    shoulderRoll,
    elbowYaw,
    elbowRoll,
    wristYaw,
    hand,
  });

  ENUM(JointLegVarieties,
  {,
    hipYawPitch, //< not a joint in the real NAO
    hipRoll,
    hipPitch,
    kneePitch,
    anklePitch,
    ankleRoll,
  });

  inline Joint combine(const Arms::Arm arm, const JointArmVarieties jointV)
  {
    static const unsigned offset[2] = { 0u, firstRightArmJoint - firstLeftArmJoint };
    return Joint(firstArmJoint + jointV + offset[arm]);
  }

  inline Joint combine(const Legs::Leg leg, const JointLegVarieties jointV)
  {
    static const unsigned offset[2] = { 0u, firstRightLegJoint - firstLeftLegJoint };
    return Joint(firstLeftLegJoint + jointV + offset[leg]);
  }

  inline Joint mirror(const Joint joint)
  {
    switch(joint)
    {
      case Joints::headYaw:
        return Joints::headYaw;
      case Joints::lShoulderPitch:
      case Joints::lHand:
        return Joint(joint - Joints::lShoulderPitch + Joints::rShoulderPitch);
      case Joints::lElbowRoll:
      case Joints::lShoulderRoll:
      case Joints::lElbowYaw:
      case Joints::lWristYaw:
        return Joint(joint - Joints::lShoulderPitch + Joints::rShoulderPitch);
      case Joints::rShoulderPitch:
      case Joints::rHand:
        return Joint(joint - Joints::rShoulderPitch + Joints::lShoulderPitch);
      case Joints::rElbowRoll:
      case Joints::rShoulderRoll:
      case Joints::rElbowYaw:
      case Joints::rWristYaw:
        return Joint(joint - Joints::rShoulderPitch + Joints::lShoulderPitch);
      case Joints::lHipYawPitch:
      case Joints::lHipPitch:
      case Joints::lKneePitch:
      case Joints::lAnklePitch:
        return Joint(joint - Joints::lHipYawPitch + Joints::rHipYawPitch);
      case Joints::lHipRoll:
      case Joints::lAnkleRoll:
        return Joint(joint - Joints::lHipYawPitch + Joints::rHipYawPitch);
      case Joints::rHipYawPitch:
      case Joints::rHipPitch:
      case Joints::rKneePitch:
      case Joints::rAnklePitch:
        return Joint(joint - Joints::rHipYawPitch + Joints::lHipYawPitch);
      case Joints::rHipRoll:
      case Joints::rAnkleRoll:
        return Joint(joint - Joints::rHipYawPitch + Joints::lHipYawPitch);
      default:
        return joint;
    }
  }

  inline bool canNegate(const Joint joint)
  {
    switch(joint)
    {
      case Joints::headYaw:
        return true;
      case Joints::lShoulderPitch:
      case Joints::lHand:
        return false;
      case Joints::lElbowRoll:
      case Joints::lShoulderRoll:
      case Joints::lElbowYaw:
      case Joints::lWristYaw:
        return true;
      case Joints::rShoulderPitch:
      case Joints::rHand:
        return false;
      case Joints::rElbowRoll:
      case Joints::rShoulderRoll:
      case Joints::rElbowYaw:
      case Joints::rWristYaw:
        return true;
      case Joints::lHipYawPitch:
      case Joints::lHipPitch:
      case Joints::lKneePitch:
      case Joints::lAnklePitch:
        return false;
      case Joints::lHipRoll:
      case Joints::lAnkleRoll:
        return true;
      case Joints::rHipYawPitch:
      case Joints::rHipPitch:
      case Joints::rKneePitch:
      case Joints::rAnklePitch:
        return false;
      case Joints::rHipRoll:
      case Joints::rAnkleRoll:
        return true;
      default:
        return false;
    }
  }
}
