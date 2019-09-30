#include "JointAngles.h"
#include "Tools/Debugging/DebugDrawings.h"

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
