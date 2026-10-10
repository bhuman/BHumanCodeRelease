#include "JointAngles.h"
#include "Debugging/Plot.h"

Angle JointAngles::mirror(Joints::Joint joint) const
{
  if(canNegate(joint))
  {
    return JointAngles::mirror(angles[Joints::mirror(joint)]);
  }
  else
    return angles[Joints::mirror(joint)];
}

void JointAngles::draw()
{
  PLOT("representation:JointAngles:headYaw", angles[Joints::headYaw].toDegrees());
  PLOT("representation:JointAngles:headPitch", angles[Joints::headPitch].toDegrees());
  PLOT("representation:JointAngles:lShoulderPitch", angles[Joints::lShoulderPitch].toDegrees());
  PLOT("representation:JointAngles:lShoulderRoll", angles[Joints::lShoulderRoll].toDegrees());
  PLOT("representation:JointAngles:lElbowPitch", angles[Joints::lElbowPitch].toDegrees());
  PLOT("representation:JointAngles:lElbowYaw", angles[Joints::lElbowYaw].toDegrees());
  PLOT("representation:JointAngles:rShoulderPitch", angles[Joints::rShoulderPitch].toDegrees());
  PLOT("representation:JointAngles:rShoulderRoll", angles[Joints::rShoulderRoll].toDegrees());
  PLOT("representation:JointAngles:rElbowPitch", angles[Joints::rElbowPitch].toDegrees());
  PLOT("representation:JointAngles:rElbowYaw", angles[Joints::rElbowYaw].toDegrees());
  PLOT("representation:JointAngles:waistYaw", angles[Joints::waistYaw].toDegrees());
  PLOT("representation:JointAngles:lHipPitch", angles[Joints::lHipPitch].toDegrees());
  PLOT("representation:JointAngles:lHipRoll", angles[Joints::lHipRoll].toDegrees());
  PLOT("representation:JointAngles:lHipYaw", angles[Joints::lHipYaw].toDegrees());
  PLOT("representation:JointAngles:lKneePitch", angles[Joints::lKneePitch].toDegrees());
  PLOT("representation:JointAngles:lAnklePitch", angles[Joints::lAnklePitch].toDegrees());
  PLOT("representation:JointAngles:lAnkleRoll", angles[Joints::lAnkleRoll].toDegrees());
  PLOT("representation:JointAngles:rHipPitch", angles[Joints::rHipPitch].toDegrees());
  PLOT("representation:JointAngles:rHipRoll", angles[Joints::rHipRoll].toDegrees());
  PLOT("representation:JointAngles:rHipYaw", angles[Joints::rHipYaw].toDegrees());
  PLOT("representation:JointAngles:rKneePitch", angles[Joints::rKneePitch].toDegrees());
  PLOT("representation:JointAngles:rAnklePitch", angles[Joints::rAnklePitch].toDegrees());
  PLOT("representation:JointAngles:rAnkleRoll", angles[Joints::rAnkleRoll].toDegrees());

  PLOT("representation:JointAngles:velocity:headYaw", velocity[Joints::headYaw].toDegrees());
  PLOT("representation:JointAngles:velocity:headPitch", velocity[Joints::headPitch].toDegrees());
  PLOT("representation:JointAngles:velocity:lShoulderPitch", velocity[Joints::lShoulderPitch].toDegrees());
  PLOT("representation:JointAngles:velocity:lShoulderRoll", velocity[Joints::lShoulderRoll].toDegrees());
  PLOT("representation:JointAngles:velocity:lElbowPitch", velocity[Joints::lElbowPitch].toDegrees());
  PLOT("representation:JointAngles:velocity:lElbowYaw", velocity[Joints::lElbowYaw].toDegrees());
  PLOT("representation:JointAngles:velocity:rShoulderPitch", velocity[Joints::rShoulderPitch].toDegrees());
  PLOT("representation:JointAngles:velocity:rShoulderRoll", velocity[Joints::rShoulderRoll].toDegrees());
  PLOT("representation:JointAngles:velocity:rElbowPitch", velocity[Joints::rElbowPitch].toDegrees());
  PLOT("representation:JointAngles:velocity:rElbowYaw", velocity[Joints::rElbowYaw].toDegrees());
  PLOT("representation:JointAngles:velocity:waistYaw", velocity[Joints::waistYaw].toDegrees());
  PLOT("representation:JointAngles:velocity:lHipPitch", velocity[Joints::lHipPitch].toDegrees());
  PLOT("representation:JointAngles:velocity:lKneePitch", velocity[Joints::lKneePitch].toDegrees());
  PLOT("representation:JointAngles:velocity:lAnklePitch", velocity[Joints::lAnklePitch].toDegrees());
  PLOT("representation:JointAngles:velocity:lAnkleRoll", velocity[Joints::lAnkleRoll].toDegrees());
  PLOT("representation:JointAngles:velocity:rHipPitch", velocity[Joints::rHipPitch].toDegrees());
  PLOT("representation:JointAngles:velocity:rHipRoll", velocity[Joints::rHipRoll].toDegrees());
  PLOT("representation:JointAngles:velocity:rHipYaw", velocity[Joints::rHipYaw].toDegrees());
  PLOT("representation:JointAngles:velocity:rKneePitch", velocity[Joints::rKneePitch].toDegrees());
  PLOT("representation:JointAngles:velocity:rAnklePitch", velocity[Joints::rAnklePitch].toDegrees());
  PLOT("representation:JointAngles:velocity:rAnkleRoll", velocity[Joints::rAnkleRoll].toDegrees());
}
