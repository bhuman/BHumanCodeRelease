/**
 * @file JointPlay.h
 * @author Philip Reichenberg
 */

#include "JointPlay.h"
#include "Debugging/Plot.h"

void JointPlay::draw() const
{
  PLOT("representation:JointPlay:play:upper:lHipYawPitch", jointState[Joints::lHipYawPitch].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:lHipYawPitch", jointState[Joints::lHipYawPitch].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:lHipRoll", jointState[Joints::lHipRoll].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:lHipRoll", jointState[Joints::lHipRoll].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:lHipPitch", jointState[Joints::lHipPitch].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:lHipPitch", jointState[Joints::lHipPitch].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:lKneePitch", jointState[Joints::lKneePitch].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:lKneePitch", jointState[Joints::lKneePitch].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:lAnklePitch", jointState[Joints::lAnklePitch].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:lAnklePitch", jointState[Joints::lAnklePitch].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:lAnkleRoll", jointState[Joints::lAnkleRoll].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:lAnkleRoll", jointState[Joints::lAnkleRoll].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:rHipRoll", jointState[Joints::rHipRoll].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:rHipRoll", jointState[Joints::rHipRoll].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:rHipPitch", jointState[Joints::rHipPitch].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:rHipPitch", jointState[Joints::rHipPitch].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:rKneePitch", jointState[Joints::rKneePitch].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:rKneePitch", jointState[Joints::rKneePitch].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:rAnklePitch", jointState[Joints::rAnklePitch].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:rAnklePitch", jointState[Joints::rAnklePitch].requestBoundary.min.toDegrees());
  PLOT("representation:JointPlay:play:upper:rAnkleRoll", jointState[Joints::rAnkleRoll].requestBoundary.max.toDegrees());
  PLOT("representation:JointPlay:play:lower:rAnkleRoll", jointState[Joints::rAnkleRoll].requestBoundary.min.toDegrees());
}

std::vector<Joints::Joint> JointPlay::getJointsWithSensorJump(const JointPlay::JointStatus status) const
{
  auto checkStatus = [status](const unsigned value)
  {
    switch(status)
    {
      case JointPlay::allFine:
        return value == 0;
      case JointPlay::sensorJump:
        return value == 1;
      case JointPlay::broken:
        return value > 2;
      case JointPlay::damaged:
        return value > 0;
      default:
        FAIL("Wrong status");
        return false;
    }
  };

  std::vector<Joints::Joint> result;
  for(std::size_t joint = 0; joint < Joints::numOfJoints; joint++)
  {
    if(checkStatus(jointState[joint].status))
      result.push_back(Joints::Joint(joint));
  }
  return result;
};
