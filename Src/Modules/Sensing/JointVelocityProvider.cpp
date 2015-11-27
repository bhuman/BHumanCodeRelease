
#include "JointVelocityProvider.h"
#include "Tools/Math/Angle.h"
#include <string.h>

using namespace std;

MAKE_MODULE(JointVelocityProvider, sensing)

JointVelocityProvider::JointVelocityProvider() : first(true)
{}

void JointVelocityProvider::update(JointVelocities &jv)
{

  if(first)
  {
    for(unsigned i = 0; i < Joints::numOfJoints; ++i)
    {
      filteredPositions[i].init(theJointAngles.angles[i], 0);
    }
    first = false;
  }

  Angle oldVelocities[Joints::numOfJoints];
  memcpy(oldVelocities, jv.velocities, sizeof(Angle) * Joints::numOfJoints);
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    jv.velocities[i] = (theJointAngles.angles[i] - lastJointData.angles[i]) / theFrameInfo.cycleTime;
    jv.accelerations[i] = (jv.velocities[i] - oldVelocities[i]) /theFrameInfo.cycleTime;

    const float oldFilteredPosition = filteredPositions[i].value;
    filteredPositions[i].predict(0, predictVariance);
    filteredPositions[i].update(theJointAngles.angles[i], updateVariance);
    jv.filteredVelocities[i] = (filteredPositions[i].value - oldFilteredPosition) / theFrameInfo.cycleTime;

  }

  lastJointData = theJointAngles;
}
