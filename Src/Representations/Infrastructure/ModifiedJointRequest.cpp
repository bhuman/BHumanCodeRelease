#include "ModifiedJointRequest.h"
#include "JointRequest.h"
#include "Debugging/Plot.h"

void ModifiedJointRequest::draw() const
{
  if(Blackboard::getInstance().exists("JointRequest"))
  {
    const JointRequest& theJointRequest = static_cast<JointRequest&>(Blackboard::getInstance()["JointRequest"]);
    PLOT("representation:ModifiedJointRequest:headYaw", Angle(theJointRequest.angles[Joints::headYaw] + angles[Joints::headYaw]).toDegrees());
    PLOT("representation:ModifiedJointRequest:headPitch", Angle(theJointRequest.angles[Joints::headPitch] + angles[Joints::headPitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:waistYaw", Angle(theJointRequest.angles[Joints::waistYaw] + angles[Joints::waistYaw]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lShoulderPitch", Angle(theJointRequest.angles[Joints::lShoulderPitch] + angles[Joints::lShoulderPitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lShoulderRoll", Angle(theJointRequest.angles[Joints::lShoulderRoll] + angles[Joints::lShoulderRoll]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lElbowYaw", Angle(theJointRequest.angles[Joints::lElbowYaw] + angles[Joints::lElbowYaw]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lElbowRoll", Angle(theJointRequest.angles[Joints::lElbowRoll] + angles[Joints::lElbowRoll]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lWristYaw", Angle(theJointRequest.angles[Joints::lWristYaw] + angles[Joints::lWristYaw]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lHand", Angle(theJointRequest.angles[Joints::lHand] + angles[Joints::lHand]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rShoulderPitch", Angle(theJointRequest.angles[Joints::rShoulderPitch] + angles[Joints::rShoulderPitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rShoulderRoll", Angle(theJointRequest.angles[Joints::rShoulderRoll] + angles[Joints::rShoulderRoll]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rElbowYaw", Angle(theJointRequest.angles[Joints::rElbowYaw] + angles[Joints::rElbowYaw]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rElbowRoll", Angle(theJointRequest.angles[Joints::rElbowRoll] + angles[Joints::rElbowRoll]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rWristYaw", Angle(theJointRequest.angles[Joints::rWristYaw] + angles[Joints::rWristYaw]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rHand", Angle(theJointRequest.angles[Joints::rHand] + angles[Joints::rHand]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lHipYawPitch", Angle(theJointRequest.angles[Joints::lHipYawPitch] + angles[Joints::lHipYawPitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lHipRoll", Angle(theJointRequest.angles[Joints::lHipRoll] + angles[Joints::lHipRoll]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lHipPitch", Angle(theJointRequest.angles[Joints::lHipPitch] + angles[Joints::lHipPitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lKneePitch", Angle(theJointRequest.angles[Joints::lKneePitch] + angles[Joints::lKneePitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lAnklePitch", Angle(theJointRequest.angles[Joints::lAnklePitch] + angles[Joints::lAnklePitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:lAnkleRoll", Angle(theJointRequest.angles[Joints::lAnkleRoll] + angles[Joints::lAnkleRoll]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rHipYawPitch", Angle(theJointRequest.angles[Joints::rHipYawPitch] + angles[Joints::rHipYawPitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rHipRoll", Angle(theJointRequest.angles[Joints::rHipRoll] + angles[Joints::rHipRoll]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rHipPitch", Angle(theJointRequest.angles[Joints::rHipPitch] + angles[Joints::rHipPitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rKneePitch", Angle(theJointRequest.angles[Joints::rKneePitch] + angles[Joints::rKneePitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rAnklePitch", Angle(theJointRequest.angles[Joints::rAnklePitch] + angles[Joints::rAnklePitch]).toDegrees());
    PLOT("representation:ModifiedJointRequest:rAnkleRoll", Angle(theJointRequest.angles[Joints::rAnkleRoll] + angles[Joints::rAnkleRoll]).toDegrees());
  }
}

ModifiedJointRequest& ModifiedJointRequest::operator=(const JointAngles& jointAngles)
{
  static_cast<JointAngles&>(*this) = jointAngles;
  return *this;
}
