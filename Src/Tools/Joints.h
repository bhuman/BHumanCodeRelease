#pragma once

#include "Enum.h"

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
    lHipYawPitch,
    lHipRoll,
    lHipPitch,
    lKneePitch,
    lAnklePitch,
    lAnkleRoll,
    rHipYawPitch, //< not a joint in the real nao
    rHipRoll,
    rHipPitch,
    rKneePitch,
    rAnklePitch,
    rAnkleRoll,
  });
}