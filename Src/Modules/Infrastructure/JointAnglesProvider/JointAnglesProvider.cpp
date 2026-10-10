#include "JointAnglesProvider.h"

MAKE_MODULE(JointAnglesProvider);

void JointAnglesProvider::update(JointAngles& jointAngles)
{
  jointAngles = theJointSensorData;

  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(jointAngles.angles[joint] != JointAngles::off && jointAngles.angles[joint] != JointAngles::ignore)
      jointAngles.angles[joint] -= theJointCalibration.offsets[joint];
  }

  jointAngles.velocity = theJointSensorData.velocity;
}
