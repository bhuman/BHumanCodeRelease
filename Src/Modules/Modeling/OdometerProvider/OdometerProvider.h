/*
 * @file OdometerProvider.h
 *
 * Declaration of module that computes some additional odometry information
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author marcel
 */

#pragma once

#include "Framework/Module.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/Odometer.h"

MODULE(OdometerProvider,
{,
  REQUIRES(OdometryData),
  PROVIDES(Odometer),
  LOADS_PARAMETERS(
  {,
    (float) sigmaAngle, ///< The angular walking odometry uncertainty
    (float) sigmaDistance, ///< The linear walking odometry uncertainty
  }),
});

/*
 * @class OdometerProvider
 *
 * Module that computes some additional odometry information
 */
class OdometerProvider : public OdometerProviderBase
{
private:
  Pose2f lastOdometryData; /**< Odometry data in last frame */

  /**
   * The method that computes the odometry information
   *
   * @param odometer The odometry information that is updated by this module.
   */
  void update(Odometer& odometer) override;
};
