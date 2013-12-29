/**
* @file OdometryOnlySelfLocator.h
*
* Declares a class that performs self-localization by adding odometry offsets.
* This is not for real self-localization but for testing and debugging.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once


#include "Tools/Module/Module.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Modeling/RobotPose.h"


MODULE(OdometryOnlySelfLocator)
  REQUIRES(OdometryData)
  PROVIDES_WITH_OUTPUT(RobotPose)
END_MODULE


/**
* @class OdometryOnlySelfLocator
*
* A new module for self-localization
*/
class OdometryOnlySelfLocator : public OdometryOnlySelfLocatorBase
{
private:
  Pose2D base;
  Pose2D referenceOdometry;

  /**
  * The method provides the robot pose
  *
  * @param robotPose The robot pose representation that is updated by this module.
  */
  void update(RobotPose& robotPose);
};
