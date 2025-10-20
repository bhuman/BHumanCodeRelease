#include "JointRequest.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Framework/Blackboard.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/RobotModel.h"

void JointRequest::draw()
{
  PLOT("representation:JointRequest:headYaw", angles[Joints::headYaw].toDegrees());
  PLOT("representation:JointRequest:headPitch", angles[Joints::headPitch].toDegrees());
  PLOT("representation:JointRequest:waistYaw", angles[Joints::waistYaw].toDegrees());
  PLOT("representation:JointRequest:lShoulderPitch", angles[Joints::lShoulderPitch].toDegrees());
  PLOT("representation:JointRequest:lShoulderRoll", angles[Joints::lShoulderRoll].toDegrees());
  PLOT("representation:JointRequest:lElbowYaw", angles[Joints::lElbowYaw].toDegrees());
  PLOT("representation:JointRequest:lElbowRoll", angles[Joints::lElbowRoll].toDegrees());
  PLOT("representation:JointRequest:lWristYaw", angles[Joints::lWristYaw].toDegrees());
  PLOT("representation:JointRequest:lHand", angles[Joints::lHand].toDegrees());
  PLOT("representation:JointRequest:rShoulderPitch", angles[Joints::rShoulderPitch].toDegrees());
  PLOT("representation:JointRequest:rShoulderRoll", angles[Joints::rShoulderRoll].toDegrees());
  PLOT("representation:JointRequest:rElbowYaw", angles[Joints::rElbowYaw].toDegrees());
  PLOT("representation:JointRequest:rElbowRoll", angles[Joints::rElbowRoll].toDegrees());
  PLOT("representation:JointRequest:rWristYaw", angles[Joints::rWristYaw].toDegrees());
  PLOT("representation:JointRequest:rHand", angles[Joints::rHand].toDegrees());
  PLOT("representation:JointRequest:lHipYawPitch", angles[Joints::lHipYawPitch].toDegrees());
  PLOT("representation:JointRequest:lHipRoll", angles[Joints::lHipRoll].toDegrees());
  PLOT("representation:JointRequest:lHipPitch", angles[Joints::lHipPitch].toDegrees());
  PLOT("representation:JointRequest:lKneePitch", angles[Joints::lKneePitch].toDegrees());
  PLOT("representation:JointRequest:lAnklePitch", angles[Joints::lAnklePitch].toDegrees());
  PLOT("representation:JointRequest:lAnkleRoll", angles[Joints::lAnkleRoll].toDegrees());
  PLOT("representation:JointRequest:rHipYawPitch", angles[Joints::rHipYawPitch].toDegrees());
  PLOT("representation:JointRequest:rHipRoll", angles[Joints::rHipRoll].toDegrees());
  PLOT("representation:JointRequest:rHipPitch", angles[Joints::rHipPitch].toDegrees());
  PLOT("representation:JointRequest:rKneePitch", angles[Joints::rKneePitch].toDegrees());
  PLOT("representation:JointRequest:rAnklePitch", angles[Joints::rAnklePitch].toDegrees());
  PLOT("representation:JointRequest:rAnkleRoll", angles[Joints::rAnkleRoll].toDegrees());

  DEBUG_DRAWING3D("representation:JointRequest:Limbs", "robot")
  {
    if(Blackboard::getInstance().exists("MassCalibration") && Blackboard::getInstance().exists("RobotDimensions"))
    {
      const MassCalibration& theMassCalibration = static_cast<MassCalibration&>(Blackboard::getInstance()["MassCalibration"]);
      const RobotDimensions& theRobotDimensions = static_cast<RobotDimensions&>(Blackboard::getInstance()["RobotDimensions"]);
      const RobotModel requestedModel(*this, theRobotDimensions, theMassCalibration);
      FOOT3D("representation:JointRequest:Limbs", requestedModel.soleLeft, true, theRobotDimensions.soleShape, ColorRGBA::blue);
      FOOT3D("representation:JointRequest:Limbs", requestedModel.soleRight, false, theRobotDimensions.soleShape, ColorRGBA::blue);
      SUBCOORDINATES3D("representation:JointRequest:Limbs", requestedModel.limbs[Limbs::wristLeft], 50.f, 50.f);
      SUBCOORDINATES3D("representation:JointRequest:Limbs", requestedModel.limbs[Limbs::wristRight], 50.f, 50.f);
    }
  }
}

JointRequest::JointRequest()
{
  angles.fill(off);
}

void JointRequest::mirror(const JointRequest& other, const bool hasSeparateHipYawJoints)
{
  JointAngles::mirror(other, hasSeparateHipYawJoints);
  stiffnessData.mirror(other.stiffnessData);
}

Angle JointRequest::mirror(Joints::Joint joint, const bool hasSeparateHipYawJoints) const
{
  return JointAngles::mirror(joint, hasSeparateHipYawJoints);
}

bool JointRequest::isValid(bool allowUseDefault) const
{
  bool isValid = true;
  for(unsigned i = 0; i < Joints::numOfJoints; i++)
    if(!std::isfinite(angles[i]))
    {
      OUTPUT_ERROR("Joint " << TypeRegistry::getEnumName(Joints::Joint(i)) << " is invalid");
      isValid = false;
    }
  return stiffnessData.isValid(allowUseDefault) && isValid;
}
