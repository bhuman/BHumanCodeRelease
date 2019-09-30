/**
 * @file JointCalibrator.cpp
 * This file implements a module with tools for calibrating leg joints.
 * @author Colin Graf
 */

#include "JointCalibrator.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Representations/Sensing/RobotModel.h"

MAKE_MODULE(JointCalibrator, infrastructure)

void JointCalibrator::update(JointCalibration& jointCalibration)
{
  bool allActive = true;
  for(int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; ++i)
    allActive &= theJointRequest.angles[i] != JointAngles::off;

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:reset")
  {
    for(int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; ++i)
      jointCalibration.offsets[i] = 0.f;

    original = jointCalibration;
    offsets.clear();
    lastOffsets.clear();
  }

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:init")
  {
    original = jointCalibration;
    offsets.clear();
    lastOffsets.clear();
  }

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:reload")
  {
    InMapFile stream("jointCalibration.cfg");
    if(stream.exists())
      stream >> jointCalibration;
    else
      OUTPUT_WARNING("jointCalibration.cfg not found.");

    original = jointCalibration;
    offsets.clear();
    lastOffsets.clear();
  }

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:capture")
  {
    if(!allActive)
      OUTPUT_TEXT("Error: capturing not possible because at least one joint is off");
    else
      for(int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; ++i)
        jointCalibration.offsets[i] += theJointAngles.angles[i] - theJointRequest.angles[i];
  }

  MODIFY_ONCE("module:JointCalibrator:offsets", offsets);
  if(offsets != lastOffsets)
  {
    if(!allActive)
      OUTPUT_TEXT("Error: setting offsets not possible because at least one joint is off");
    else
    {
      const RobotModel robotModel(theJointRequest, theRobotDimensions, theMassCalibration);
      const Pose3f leftFootRotated = robotModel.soleLeft * Rotation::Euler::fromAngles(offsets.leftFoot.rotation);
      const Pose3f rightFootRotated = robotModel.soleRight * Rotation::Euler::fromAngles(offsets.rightFoot.rotation);
      const Pose3f leftOffset = leftFootRotated + offsets.leftFoot.translation;
      const Pose3f rightOffset = rightFootRotated + offsets.rightFoot.translation;
      JointAngles jointAngles;
      if(!InverseKinematic::calcLegJoints(leftOffset, rightOffset, offsets.bodyRotation.cast<float>(), jointAngles, theRobotDimensions))
        OUTPUT_TEXT("Warning: at least one foot position unreachable");

      for(int i = Joints::lHipYawPitch; i <= Joints::rAnkleRoll; ++i)
        jointCalibration.offsets[i] = original.offsets[i] + jointAngles.angles[i] - theJointRequest.angles[i];
    }

    lastOffsets = offsets;
  }
}

void JointCalibrator::setOffsets(Angle x, Angle y)
{
  Offsets buff;
  buff.bodyRotation.x() = x;
  buff.bodyRotation.y() = y;
  OUTPUT_TEXT("");
  OUTPUT_TEXT("mr JointCalibration JointCalibrator");
  OUTPUT_TEXT("dr module:JointCalibrator:init");
  streamOffsets("offsets", buff);
  OUTPUT_TEXT("");
  OUTPUT_TEXT("save representation:JointCalibration");
  OUTPUT_TEXT("save representation:CameraCalibration");
}

void JointCalibrator::streamOffsets(const std::string representationName, const Streamable& representation)
{
  OutMapMemory memory(true, 1024);
  memory << representation;
  std::string command = "set module:JointCalibrator:" + representationName + " " + memory.data();
  OUTPUT_TEXT(command);
}
