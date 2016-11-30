#include "JointAngles.h"
#include "Tools/Debugging/DebugDrawings.h"

Angle JointAngles::mirror(Joints::Joint joint) const
{
  switch(joint)
  {
    case Joints::headYaw:
      return mirror(angles[Joints::headYaw]);
    case Joints::lShoulderPitch:
    case Joints::lHand:
      return angles[joint - Joints::lShoulderPitch + Joints::rShoulderPitch];
    case Joints::lElbowRoll:
    case Joints::lShoulderRoll:
    case Joints::lElbowYaw:
    case Joints::lWristYaw:
      return mirror(angles[joint - Joints::lShoulderPitch + Joints::rShoulderPitch]);
    case Joints::rShoulderPitch:
    case Joints::rHand:
      return angles[joint - Joints::rShoulderPitch + Joints::lShoulderPitch];
    case Joints::rElbowRoll:
    case Joints::rShoulderRoll:
    case Joints::rElbowYaw:
    case Joints::rWristYaw:
      return mirror(angles[joint - Joints::rShoulderPitch + Joints::lShoulderPitch]);
    case Joints::lHipYawPitch:
    case Joints::lHipPitch:
    case Joints::lKneePitch:
    case Joints::lAnklePitch:
      return angles[joint - Joints::lHipYawPitch + Joints::rHipYawPitch];
    case Joints::lHipRoll:
    case Joints::lAnkleRoll:
      return mirror(angles[joint - Joints::lHipYawPitch + Joints::rHipYawPitch]);
    case Joints::rHipYawPitch:
    case Joints::rHipPitch:
    case Joints::rKneePitch:
    case Joints::rAnklePitch:
      return angles[joint - Joints::rHipYawPitch + Joints::lHipYawPitch];
    case Joints::rHipRoll:
    case Joints::rAnkleRoll:
      return mirror(angles[joint - Joints::rHipYawPitch + Joints::lHipYawPitch]);
    default:
      return angles[joint];
  }
}

void JointAngles::draw()
{
  PLOT("representation:JointAngles:headYaw", angles[Joints::headYaw].toDegrees());
  PLOT("representation:JointAngles:headPitch", angles[Joints::headPitch].toDegrees());
  PLOT("representation:JointAngles:lShoulderPitch", angles[Joints::lShoulderPitch].toDegrees());
  PLOT("representation:JointAngles:lShoulderRoll", angles[Joints::lShoulderRoll].toDegrees());
  PLOT("representation:JointAngles:lElbowYaw", angles[Joints::lElbowYaw].toDegrees());
  PLOT("representation:JointAngles:lElbowRoll", angles[Joints::lElbowRoll].toDegrees());
  PLOT("representation:JointAngles:lWristYaw", angles[Joints::lWristYaw].toDegrees());
  PLOT("representation:JointAngles:lHand", angles[Joints::lHand].toDegrees());
  PLOT("representation:JointAngles:rShoulderPitch", angles[Joints::rShoulderPitch].toDegrees());
  PLOT("representation:JointAngles:rShoulderRoll", angles[Joints::rShoulderRoll].toDegrees());
  PLOT("representation:JointAngles:rElbowYaw", angles[Joints::rElbowYaw].toDegrees());
  PLOT("representation:JointAngles:rElbowRoll", angles[Joints::rElbowRoll].toDegrees());
  PLOT("representation:JointAngles:rWristYaw", angles[Joints::rWristYaw].toDegrees());
  PLOT("representation:JointAngles:rHand", angles[Joints::rHand].toDegrees());
  PLOT("representation:JointAngles:lHipYawPitch", angles[Joints::lHipYawPitch].toDegrees());
  PLOT("representation:JointAngles:lHipRoll", angles[Joints::lHipRoll].toDegrees());
  PLOT("representation:JointAngles:lHipPitch", angles[Joints::lHipPitch].toDegrees());
  PLOT("representation:JointAngles:lKneePitch", angles[Joints::lKneePitch].toDegrees());
  PLOT("representation:JointAngles:lAnklePitch", angles[Joints::lAnklePitch].toDegrees());
  PLOT("representation:JointAngles:lAnkleRoll", angles[Joints::lAnkleRoll].toDegrees());
  PLOT("representation:JointAngles:rHipYawPitch", angles[Joints::rHipYawPitch].toDegrees());
  PLOT("representation:JointAngles:rHipRoll", angles[Joints::rHipRoll].toDegrees());
  PLOT("representation:JointAngles:rHipPitch", angles[Joints::rHipPitch].toDegrees());
  PLOT("representation:JointAngles:rKneePitch", angles[Joints::rKneePitch].toDegrees());
  PLOT("representation:JointAngles:rAnklePitch", angles[Joints::rAnklePitch].toDegrees());
  PLOT("representation:JointAngles:rAnkleRoll", angles[Joints::rAnkleRoll].toDegrees());
}
