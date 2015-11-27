/*
 * @file OdometerProvider.h
 *
 * Declaration of module that computes some additional odometry information
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 * @author marcel
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/Odometer.h"

MODULE(OdometerProvider,
{,
  REQUIRES(OdometryData),
  PROVIDES(Odometer),
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
  void update(Odometer& odometer);
};
