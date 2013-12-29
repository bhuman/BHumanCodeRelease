/**
* @file JointCalibrator.cpp
* This file implements a module with tools for calibrating leg joints.
* @author Colin Graf
*/

#include "JointCalibrator.h"
#include "Tools/InverseKinematic.h"
#include "Representations/Sensing/RobotModel.h"

MAKE_MODULE(JointCalibrator, Motion Infrastructure)

void JointCalibrator::update(JointCalibration& jointCalibration)
{
  bool allActive = true;
  for(int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i)
    allActive &= theJointRequest.angles[i] != JointData::off;

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:reset",
  {
    Global::getDebugRequestTable().disable("module:JointCalibrator:reset");
    for(int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i)
      jointCalibration.joints[i].offset = 0;

    offsets.clear();
    lastOffsets.clear();
  });

  DEBUG_RESPONSE_ONCE("module:JointCalibrator:capture",
  {
    Global::getDebugRequestTable().disable("module:JointCalibrator:capture");

    if(!allActive)
      OUTPUT(idText, text, "Error: capturing not possible because at least one joint is off");
    else
      for(int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i)
        jointCalibration.joints[i].offset += theJointData.angles[i] - theJointRequest.angles[i];
  });

  MODIFY("module:JointCalibrator:offsets", offsets);
  if(offsets != lastOffsets)
  {
    if(!allActive)
      OUTPUT(idText, text, "Error: setting offsets not possible because at least one joint is off");
    else
    {
      Offsets additionalOffsets = offsets;
      additionalOffsets -= lastOffsets;
      Pose3D rotationOffset(RotationMatrix(additionalOffsets.bodyRotation));
      Pose3D leftOffset(RotationMatrix(additionalOffsets.leftFoot.rotation), additionalOffsets.leftFoot.translation);
      Pose3D rightOffset(RotationMatrix(additionalOffsets.rightFoot.rotation), additionalOffsets.rightFoot.translation);
      RobotModel robotModel(theJointRequest, theRobotDimensions, theMassCalibration);
      JointData jointData;
      InverseKinematic::calcLegJoints(Pose3D(rotationOffset).conc(robotModel.limbs[MassCalibration::footLeft]).conc(leftOffset),
                                      Pose3D(rotationOffset).conc(robotModel.limbs[MassCalibration::footRight]).conc(rightOffset), jointData, theRobotDimensions);

      for(int i = JointData::LHipYawPitch; i <= JointData::RAnkleRoll; ++i)
        jointCalibration.joints[i].offset += jointData.angles[i] - theJointRequest.angles[i];
    }

    lastOffsets = offsets;
  }
}
