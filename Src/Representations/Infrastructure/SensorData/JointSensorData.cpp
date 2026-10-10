/**
 * @file JointSensorData.cpp
 */

#include "JointSensorData.h"
#include "Debugging/Plot.h"

void JointSensorData::draw() const
{
  PLOT("representation:JointSensorData:torque:headYaw", torque[Joints::headYaw]);
  PLOT("representation:JointSensorData:torque:headPitch", torque[Joints::headPitch]);
  PLOT("representation:JointSensorData:torque:lShoulderPitch", torque[Joints::lShoulderPitch]);
  PLOT("representation:JointSensorData:torque:lShoulderRoll", torque[Joints::lShoulderRoll]);
  PLOT("representation:JointSensorData:torque:lElbowPitch", torque[Joints::lElbowPitch]);
  PLOT("representation:JointSensorData:torque:lElbowYaw", torque[Joints::lElbowYaw]);
  PLOT("representation:JointSensorData:torque:rShoulderPitch", torque[Joints::rShoulderPitch]);
  PLOT("representation:JointSensorData:torque:rShoulderRoll", torque[Joints::rShoulderRoll]);
  PLOT("representation:JointSensorData:torque:rElbowPitch", torque[Joints::rElbowPitch]);
  PLOT("representation:JointSensorData:torque:rElbowYaw", torque[Joints::rElbowYaw]);
  PLOT("representation:JointSensorData:torque:waistYaw", torque[Joints::waistYaw]);
  PLOT("representation:JointSensorData:torque:lHipPitch", torque[Joints::lHipPitch]);
  PLOT("representation:JointSensorData:torque:lHipRoll", torque[Joints::lHipRoll]);
  PLOT("representation:JointSensorData:torque:lHipYaw", torque[Joints::lHipYaw]);
  PLOT("representation:JointSensorData:torque:lKneePitch", torque[Joints::lKneePitch]);
  PLOT("representation:JointSensorData:torque:lAnklePitch", torque[Joints::lAnklePitch]);
  PLOT("representation:JointSensorData:torque:lAnkleRoll", torque[Joints::lAnkleRoll]);
  PLOT("representation:JointSensorData:torque:rHipPitch", torque[Joints::rHipPitch]);
  PLOT("representation:JointSensorData:torque:rHipRoll", torque[Joints::rHipRoll]);
  PLOT("representation:JointSensorData:torque:rHipYaw", torque[Joints::rHipYaw]);
  PLOT("representation:JointSensorData:torque:rKneePitch", torque[Joints::rKneePitch]);
  PLOT("representation:JointSensorData:torque:rAnklePitch", torque[Joints::rAnklePitch]);
  PLOT("representation:JointSensorData:torque:rAnkleRoll", torque[Joints::rAnkleRoll]);
}
