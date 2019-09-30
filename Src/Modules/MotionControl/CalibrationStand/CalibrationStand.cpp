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
  StandArmRequest targetJoints = StandArmRequest();

  targetJoints.angles[Joints::lShoulderPitch] = 90_deg;
  targetJoints.angles[Joints::lShoulderRoll] = jointCalibrationMode ? 70_deg : 10_deg;
  targetJoints.angles[Joints::lElbowYaw] = 0_deg;
  targetJoints.angles[Joints::lElbowRoll] = 0_deg;
  targetJoints.angles[Joints::lWristYaw] = -90_deg;
  targetJoints.angles[Joints::lHand] = 0;
  targetJoints.angles[Joints::rShoulderPitch] = 90_deg;
  targetJoints.angles[Joints::rShoulderRoll] = jointCalibrationMode ? -70_deg : -10_deg;
  targetJoints.angles[Joints::rElbowYaw] = 0_deg;
  targetJoints.angles[Joints::rElbowRoll] = 0_deg;
  targetJoints.angles[Joints::rWristYaw] = 90_deg;
  targetJoints.angles[Joints::rHand] = 0;

  if(needToInterpolateArm)
  {
    if(timestampArm == 0)
    {
      timestampArm = theFrameInfo.time - static_cast<unsigned int>(Constants::motionCycleTime);
      startJointsArm.angles = theJointAngles.angles;
    }
    float ratio = std::min(1.f, theFrameInfo.getTimeSince(timestampArm) / interpolationTimeArm);
    needToInterpolateArm = ratio < 1.f;
    MotionUtilities::interpolate(startJointsArm, targetJoints, ratio, standArmRequest, theJointAngles);
  }
  else
    standArmRequest.angles = targetJoints.angles;

  standArmRequest.stiffnessData.stiffnesses.fill(armStiffness);
}

void CalibrationStand::update(StandLegRequest& standLegRequest)
{
  DEBUG_RESPONSE_ONCE("module:CalibrationStand:height")
  {
    OUTPUT_TEXT("Robot is standing at a height of " << standHeight << " mm.");
  }
  StandLegRequest targetJoints = StandLegRequest();
  const JointAngles prevJointAngles = standLegRequest;
  bool reachable = InverseKinematic::calcLegJoints(Pose3f(Vector3f(0.f, footOffsetY, -standHeight)), Pose3f(Vector3f(0, -footOffsetY, -standHeight)),  Vector2f::Zero(), targetJoints, theRobotDimensions);

  if(needToInterpolateLeg)
  {
    if(timestampLeg == 0)
    {
      timestampLeg = theFrameInfo.time - static_cast<unsigned int>(Constants::motionCycleTime);
      startJointsLeg.angles = theJointAngles.angles;
    }
    float ratio = std::min(1.f, theFrameInfo.getTimeSince(timestampLeg) / interpolationTimeLeg);
    needToInterpolateLeg = ratio < 1.f;
    MotionUtilities::interpolate(startJointsLeg, targetJoints, ratio, standLegRequest, theJointAngles);
  }
  else
    standLegRequest.angles = targetJoints.angles;

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
