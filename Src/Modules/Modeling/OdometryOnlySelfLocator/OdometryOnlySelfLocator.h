/**
 * @file OdometryOnlySelfLocator.h
 *
 * Declares a class that performs self-localization by adding odometry offsets.
 * This is not for real self-localization but for testing and debugging.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Odometer.h"

MODULE(OdometryOnlySelfLocator,
{,
  REQUIRES(OdometryData),
  REQUIRES(Odometer),
  PROVIDES(RobotPose),
});

/**
 * @class OdometryOnlySelfLocator
 *
 * A new module for self-localization
 */
class OdometryOnlySelfLocator : public OdometryOnlySelfLocatorBase
{
private:
  Pose2f base;
  Pose2f referenceOdometry;

  /**
   * The method provides the robot pose
   *
   * @param robotPose The robot pose representation that is updated by this module.
   */
  void update(RobotPose& robotPose) override;
};
