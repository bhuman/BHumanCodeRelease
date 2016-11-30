
#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OdometryAnalyzation.h"

MODULE(OdometryAnalyzer,
{,
  REQUIRES(OdometryData),
  PROVIDES(Odometer),
  PROVIDES(OdometryAnalyzation),
  LOADS_PARAMETERS(
  {,
    (float) sigmaAngle, ///< The angular walking odometry uncertainty
    (float) sigmaDistance, ///< The linear walking odometry uncertainty
    (Vector2f) startPosition,
  }),
});

class OdometryAnalyzer : public OdometryAnalyzerBase
{
public:
  OdometryAnalyzer();

private:
  Pose2f lastOdometryData;
  Matrix3f covariance;
  Pose2f firstOdometry = Pose2f(startPosition);
  Pose2f stackingOdometry = Pose2f(startPosition);

  void update(Odometer& odometer);
  void update(OdometryAnalyzation& analyzation);
};
