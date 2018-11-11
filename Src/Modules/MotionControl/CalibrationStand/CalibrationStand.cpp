#include "CalibrationStand.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Debugging/Modify.h"
#include <cstring>

MAKE_MODULE(CalibrationStand, motionControl)

void CalibrationStand::update(StandArmRequest& standArmRequest)
{
  bool jointCalibrationMode = false;
  MODIFY("module:CalibrationStand:jointCalibrationMode", jointCalibrationMode);

  standArmRequest.angles[Joints::lShoulderPitch] = 90_deg;
  standArmRequest.angles[Joints::lShoulderRoll] = jointCalibrationMode ? 70_deg : 10_deg;
  standArmRequest.angles[Joints::lElbowYaw] = 0_deg;
  standArmRequest.angles[Joints::lElbowRoll] = 0_deg;
  standArmRequest.angles[Joints::lWristYaw] = -90_deg;
  standArmRequest.angles[Joints::lHand] = 0;
  standArmRequest.angles[Joints::rShoulderPitch] = 90_deg;
  standArmRequest.angles[Joints::rShoulderRoll] = jointCalibrationMode ? -70_deg : -10_deg;
  standArmRequest.angles[Joints::rElbowYaw] = 0_deg;
  standArmRequest.angles[Joints::rElbowRoll] = 0_deg;
  standArmRequest.angles[Joints::rWristYaw] = 90_deg;
  standArmRequest.angles[Joints::rHand] = 0;

  standArmRequest.stiffnessData.stiffnesses.fill(30);
}

void CalibrationStand::update(StandLegRequest& standLegRequest)
{
  DEBUG_RESPONSE_ONCE("module:CalibrationStand:height")
  {
    OUTPUT_TEXT("Robot is standing at a height of " << standHeight << " mm.");
  }

  const JointAngles prevJointAngles = standLegRequest;
  bool reachable = InverseKinematic::calcLegJoints(Pose3f(Vector3f(0.f, footOffsetY, -standHeight)), Pose3f(Vector3f(0, -footOffsetY, -standHeight)),  Vector2f::Zero(), standLegRequest, theRobotDimensions);
  if(!reachable && prevJointAngles.angles != standLegRequest.angles)
  {
    OUTPUT_TEXT("Warning: at least one foot pose unreachable");
  }

  DEBUG_RESPONSE("module:CalibrationStand:head")
  {
    standLegRequest.angles[Joints::headPitch] = JointAngles::ignore;
    standLegRequest.angles[Joints::headYaw] = JointAngles::ignore;
  }

  standLegRequest.stiffnessData.stiffnesses.fill(legStiffness);
}
