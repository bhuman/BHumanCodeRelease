#include "JointAnglesProvider.h"

MAKE_MODULE(JointAnglesProvider, motionInfrastructure)

void JointAnglesProvider::update(JointAngles& jointAngles)
{
  jointAngles = theJointSensorData;
  if(!theRobotInfo.hasFeature(RobotInfo::wristYaws))
  {
    jointAngles.angles[Joints::lWristYaw] = -90_deg;
    jointAngles.angles[Joints::rWristYaw] = 90_deg;
  }
}