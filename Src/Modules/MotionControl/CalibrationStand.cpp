#include "CalibrationStand.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Motion/InverseKinematic.h"
#include <cstring>

MAKE_MODULE(CalibrationStand, motionControl)

void CalibrationStand::update(StandOutput& standOutput)
{
  DEBUG_RESPONSE_ONCE("module:CalibrationStand:height")
  {
    OUTPUT_TEXT("Robot is standing at a height of " << standHeight << " mm.");
  }

  JointAngles jointAngles;

  // Set head joints
  if(useRequestedHeadAngle)
  {
    jointAngles.angles[Joints::headPitch] = theHeadJointRequest.tilt;
    jointAngles.angles[Joints::headYaw] = theHeadJointRequest.pan;
  }
  else
  {
    jointAngles.angles[Joints::headPitch] = 0_deg;
    jointAngles.angles[Joints::headYaw] = 0_deg;
  }

  // Set arm joints
  jointAngles.angles[Joints::lShoulderPitch] = 90_deg;
  jointAngles.angles[Joints::lShoulderRoll] = 10_deg;
  jointAngles.angles[Joints::lElbowYaw] = -90_deg;
  jointAngles.angles[Joints::lElbowRoll] = 0_deg;
  jointAngles.angles[Joints::rShoulderPitch] = 90_deg;
  jointAngles.angles[Joints::rShoulderRoll] = -10_deg;
  jointAngles.angles[Joints::rElbowYaw] = 90_deg;
  jointAngles.angles[Joints::rElbowRoll] = 0_deg;

  // Set leg joints
  bool reachable = InverseKinematic::calcLegJoints(Pose3f(Vector3f(0.f, footOffsetY, -standHeight)), Pose3f(Vector3f(0, -footOffsetY, -standHeight)), Vector2f::Zero(), jointAngles, theRobotDimensions);
  if(!reachable && standOutput.angles != jointAngles.angles)
  {
    OUTPUT_TEXT("Warning: at least one foot pose unreachable");
  }
  static_cast<JointAngles&>(standOutput) = jointAngles;

  // Set head stiffnesses
  standOutput.stiffnessData.stiffnesses[Joints::headYaw] = 50;
  standOutput.stiffnessData.stiffnesses[Joints::headPitch] = 50;

  // Set arm stiffnesses
  for(int i = Joints::lShoulderPitch; i <= Joints::rHand; ++i)
  {
    standOutput.stiffnessData.stiffnesses[i] = 20;
  }

  // Set leg stiffnesses
  for(int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; ++i)
  {
    standOutput.stiffnessData.stiffnesses[i] = 70;
  }
}
