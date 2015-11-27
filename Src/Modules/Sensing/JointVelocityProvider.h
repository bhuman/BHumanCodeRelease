#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/JointVelocities.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Math/Kalman.h"
#include "Platform/BHAssert.h"

MODULE(JointVelocityProvider,
{,
  REQUIRES(JointAngles),
  REQUIRES(FrameInfo),
  PROVIDES(JointVelocities),
  DEFINES_PARAMETERS(
  {,
    (float) (0.1f) predictVariance,/**<Variances for the kalmanfilter used to filter the positions */
    (float) (0.9f) updateVariance,
  }),
});

class JointVelocityProvider: public JointVelocityProviderBase
{
private:
  void update(JointVelocities& jv);
public:
  JointVelocityProvider();
private:
  JointAngles lastJointData;
  Kalman<> filteredPositions[Joints::numOfJoints];
  bool first;
};
