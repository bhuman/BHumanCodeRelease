#include "JointVelocities.h"
#include "Tools/Debugging/DebugDrawings.h"

void JointVelocities::draw() const
{
  DECLARE_PLOT("representation:JointVelocities:headYaw");
  DECLARE_PLOT("representation:JointVelocities:headPitch");
  DECLARE_PLOT("representation:JointVelocities:lShoulderPitch");
  DECLARE_PLOT("representation:JointVelocities:lShoulderRoll");
  DECLARE_PLOT("representation:JointVelocities:lElbowYaw");
  DECLARE_PLOT("representation:JointVelocities:lElbowRoll");
  DECLARE_PLOT("representation:JointVelocities:rShoulderPitch");
  DECLARE_PLOT("representation:JointVelocities:rShoulderRoll");
  DECLARE_PLOT("representation:JointVelocities:rElbowYaw");
  DECLARE_PLOT("representation:JointVelocities:rElbowRoll");
  DECLARE_PLOT("representation:JointVelocities:lHipYawPitch");
  DECLARE_PLOT("representation:JointVelocities:lHipRoll");
  DECLARE_PLOT("representation:JointVelocities:lHipPitch");
  DECLARE_PLOT("representation:JointVelocities:lKneePitch");
  DECLARE_PLOT("representation:JointVelocities:lAnklePitch");
  DECLARE_PLOT("representation:JointVelocities:lAnkleRoll");
  DECLARE_PLOT("representation:JointVelocities:rHipYawPitch");
  DECLARE_PLOT("representation:JointVelocities:rHipRoll");
  DECLARE_PLOT("representation:JointVelocities:rHipPitch");
  DECLARE_PLOT("representation:JointVelocities:rKneePitch");
  DECLARE_PLOT("representation:JointVelocities:rAnklePitch");
  DECLARE_PLOT("representation:JointVelocities:rAnkleRoll");

  DECLARE_PLOT("representation:JointVelocities:acceleration:headYaw");
  DECLARE_PLOT("representation:JointVelocities:acceleration:headPitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lShoulderPitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lShoulderRoll");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lElbowYaw");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lElbowRoll");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rShoulderPitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rShoulderRoll");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rElbowYaw");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rElbowRoll");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lHipYawPitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lHipRoll");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lHipPitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lKneePitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lAnklePitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:lAnkleRoll");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rHipYawPitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rHipRoll");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rHipPitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rKneePitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rAnklePitch");
  DECLARE_PLOT("representation:JointVelocities:acceleration:rAnkleRoll");

  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:headYaw");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:headPitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lShoulderPitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lShoulderRoll");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lElbowYaw");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lElbowRoll");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rShoulderPitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rShoulderRoll");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rElbowYaw");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rElbowRoll");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lHipYawPitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lHipRoll");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lHipPitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lKneePitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lAnklePitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:lAnkleRoll");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rHipYawPitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rHipRoll");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rHipPitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rKneePitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rAnklePitch");
  DECLARE_PLOT("representation:JointVelocities:filteredVelocity:rAnkleRoll");

  PLOT("representation:JointVelocities:headYaw", velocities[Joints::headYaw]);
  PLOT("representation:JointVelocities:headPitch", velocities[Joints::headPitch]);
  PLOT("representation:JointVelocities:lShoulderPitch", velocities[Joints::lShoulderPitch]);
  PLOT("representation:JointVelocities:lShoulderRoll", velocities[Joints::lShoulderRoll]);
  PLOT("representation:JointVelocities:lElbowYaw", velocities[Joints::lElbowYaw]);
  PLOT("representation:JointVelocities:lElbowRoll", velocities[Joints::lElbowRoll]);
  PLOT("representation:JointVelocities:rShoulderPitch", velocities[Joints::rShoulderPitch]);
  PLOT("representation:JointVelocities:rShoulderRoll", velocities[Joints::rShoulderRoll]);
  PLOT("representation:JointVelocities:rElbowYaw", velocities[Joints::rElbowYaw]);
  PLOT("representation:JointVelocities:rElbowRoll", velocities[Joints::rElbowRoll]);
  PLOT("representation:JointVelocities:lHipYawPitch", velocities[Joints::lHipYawPitch]);
  PLOT("representation:JointVelocities:lHipRoll", velocities[Joints::lHipRoll]);
  PLOT("representation:JointVelocities:lHipPitch", velocities[Joints::lHipPitch]);
  PLOT("representation:JointVelocities:lKneePitch", velocities[Joints::lKneePitch]);
  PLOT("representation:JointVelocities:lAnklePitch", velocities[Joints::lAnklePitch]);
  PLOT("representation:JointVelocities:lAnkleRoll", velocities[Joints::lAnkleRoll]);
  PLOT("representation:JointVelocities:rHipYawPitch", velocities[Joints::rHipYawPitch]);
  PLOT("representation:JointVelocities:rHipRoll", velocities[Joints::rHipRoll]);
  PLOT("representation:JointVelocities:rHipPitch", velocities[Joints::rHipPitch]);
  PLOT("representation:JointVelocities:rKneePitch", velocities[Joints::rKneePitch]);
  PLOT("representation:JointVelocities:rAnklePitch", velocities[Joints::rAnklePitch]);
  PLOT("representation:JointVelocities:rAnkleRoll", velocities[Joints::rAnkleRoll]);

  PLOT("representation:JointVelocities:filteredVelocity:headYaw", filteredVelocities[Joints::headYaw]);
  PLOT("representation:JointVelocities:filteredVelocity:headPitch", filteredVelocities[Joints::headPitch]);
  PLOT("representation:JointVelocities:filteredVelocity:lShoulderPitch", filteredVelocities[Joints::lShoulderPitch]);
  PLOT("representation:JointVelocities:filteredVelocity:lShoulderRoll", filteredVelocities[Joints::lShoulderRoll]);
  PLOT("representation:JointVelocities:filteredVelocity:lElbowYaw", filteredVelocities[Joints::lElbowYaw]);
  PLOT("representation:JointVelocities:filteredVelocity:lElbowRoll", filteredVelocities[Joints::lElbowRoll]);
  PLOT("representation:JointVelocities:filteredVelocity:rShoulderPitch", filteredVelocities[Joints::rShoulderPitch]);
  PLOT("representation:JointVelocities:filteredVelocity:rShoulderRoll", filteredVelocities[Joints::rShoulderRoll]);
  PLOT("representation:JointVelocities:filteredVelocity:rElbowYaw", filteredVelocities[Joints::rElbowYaw]);
  PLOT("representation:JointVelocities:filteredVelocity:rElbowRoll", filteredVelocities[Joints::rElbowRoll]);
  PLOT("representation:JointVelocities:filteredVelocity:lHipYawPitch", filteredVelocities[Joints::lHipYawPitch]);
  PLOT("representation:JointVelocities:filteredVelocity:lHipRoll", filteredVelocities[Joints::lHipRoll]);
  PLOT("representation:JointVelocities:filteredVelocity:lHipPitch", filteredVelocities[Joints::lHipPitch]);
  PLOT("representation:JointVelocities:filteredVelocity:lKneePitch", filteredVelocities[Joints::lKneePitch]);
  PLOT("representation:JointVelocities:filteredVelocity:lAnklePitch", filteredVelocities[Joints::lAnklePitch]);
  PLOT("representation:JointVelocities:filteredVelocity:lAnkleRoll", filteredVelocities[Joints::lAnkleRoll]);
  PLOT("representation:JointVelocities:filteredVelocity:rHipYawPitch", filteredVelocities[Joints::rHipYawPitch]);
  PLOT("representation:JointVelocities:filteredVelocity:rHipRoll", filteredVelocities[Joints::rHipRoll]);
  PLOT("representation:JointVelocities:filteredVelocity:rHipPitch", filteredVelocities[Joints::rHipPitch]);
  PLOT("representation:JointVelocities:filteredVelocity:rKneePitch", filteredVelocities[Joints::rKneePitch]);
  PLOT("representation:JointVelocities:filteredVelocity:rAnklePitch", filteredVelocities[Joints::rAnklePitch]);
  PLOT("representation:JointVelocities:filteredVelocity:rAnkleRoll", filteredVelocities[Joints::rAnkleRoll]);

  PLOT("representation:JointVelocities:acceleration:headYaw", accelerations[Joints::headYaw]);
  PLOT("representation:JointVelocities:acceleration:headPitch", accelerations[Joints::headPitch]);
  PLOT("representation:JointVelocities:acceleration:lShoulderPitch", accelerations[Joints::lShoulderPitch]);
  PLOT("representation:JointVelocities:acceleration:lShoulderRoll", accelerations[Joints::lShoulderRoll]);
  PLOT("representation:JointVelocities:acceleration:lElbowYaw", accelerations[Joints::lElbowYaw]);
  PLOT("representation:JointVelocities:acceleration:lElbowRoll", accelerations[Joints::lElbowRoll]);
  PLOT("representation:JointVelocities:acceleration:rShoulderPitch", accelerations[Joints::rShoulderPitch]);
  PLOT("representation:JointVelocities:acceleration:rShoulderRoll", accelerations[Joints::rShoulderRoll]);
  PLOT("representation:JointVelocities:acceleration:rElbowYaw", accelerations[Joints::rElbowYaw]);
  PLOT("representation:JointVelocities:acceleration:rElbowRoll", accelerations[Joints::rElbowRoll]);
  PLOT("representation:JointVelocities:acceleration:lHipYawPitch", accelerations[Joints::lHipYawPitch]);
  PLOT("representation:JointVelocities:acceleration:lHipRoll", accelerations[Joints::lHipRoll]);
  PLOT("representation:JointVelocities:acceleration:lHipPitch", accelerations[Joints::lHipPitch]);
  PLOT("representation:JointVelocities:acceleration:lKneePitch", accelerations[Joints::lKneePitch]);
  PLOT("representation:JointVelocities:acceleration:lAnklePitch", accelerations[Joints::lAnklePitch]);
  PLOT("representation:JointVelocities:acceleration:lAnkleRoll", accelerations[Joints::lAnkleRoll]);
  PLOT("representation:JointVelocities:acceleration:rHipYawPitch", accelerations[Joints::rHipYawPitch]);
  PLOT("representation:JointVelocities:acceleration:rHipRoll", accelerations[Joints::rHipRoll]);
  PLOT("representation:JointVelocities:acceleration:rHipPitch", accelerations[Joints::rHipPitch]);
  PLOT("representation:JointVelocities:acceleration:rKneePitch", accelerations[Joints::rKneePitch]);
  PLOT("representation:JointVelocities:acceleration:rAnklePitch", accelerations[Joints::rAnklePitch]);
  PLOT("representation:JointVelocities:acceleration:rAnkleRoll", accelerations[Joints::rAnkleRoll]);

  DECLARE_PLOT("representation:JointVelocities:squareSum");
  float sum = 0.0f;
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    sum += velocities[i] * velocities[i];
  }
  PLOT("representation:JointVelocities:squareSum", sum);
}
