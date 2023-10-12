/**
 * @file JointPlayTranslation.cpp
 * @author Philip Reichenberg
 */

#include "JointPlayTranslation.h"
#include "Debugging/Plot.h"

void JointPlayTranslation::draw() const
{
  PLOT("representation:JointPlayTranslation:headYaw", jointPlayState[Joints::headYaw].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:headPitch", jointPlayState[Joints::headPitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lShoulderPitch", jointPlayState[Joints::lShoulderPitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lShoulderRoll", jointPlayState[Joints::lShoulderRoll].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lElbowYaw", jointPlayState[Joints::lElbowYaw].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lElbowRoll", jointPlayState[Joints::lElbowRoll].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lWristYaw", jointPlayState[Joints::lWristYaw].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lHand", jointPlayState[Joints::lHand].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rShoulderPitch", jointPlayState[Joints::rShoulderPitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rShoulderRoll", jointPlayState[Joints::rShoulderRoll].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rElbowYaw", jointPlayState[Joints::rElbowYaw].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rElbowRoll", jointPlayState[Joints::rElbowRoll].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rWristYaw", jointPlayState[Joints::rWristYaw].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rHand", jointPlayState[Joints::rHand].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lHipYawPitch", jointPlayState[Joints::lHipYawPitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lHipRoll", jointPlayState[Joints::lHipRoll].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lHipPitch", jointPlayState[Joints::lHipPitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lKneePitch", jointPlayState[Joints::lKneePitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lAnklePitch", jointPlayState[Joints::lAnklePitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:lAnkleRoll", jointPlayState[Joints::lAnkleRoll].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rHipYawPitch", jointPlayState[Joints::rHipYawPitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rHipRoll", jointPlayState[Joints::rHipRoll].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rHipPitch", jointPlayState[Joints::rHipPitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rKneePitch", jointPlayState[Joints::rKneePitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rAnklePitch", jointPlayState[Joints::rAnklePitch].offset.toDegrees());
  PLOT("representation:JointPlayTranslation:rAnkleRoll", jointPlayState[Joints::rAnkleRoll].offset.toDegrees());
}
