/**
* @file OdometryOnlySelfLocator.cpp
*
* Declares a class that performs self-localization by adding odometry offsets.
* This is not for real self-localization but for testing and debugging.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "OdometryOnlySelfLocator.h"

void OdometryOnlySelfLocator::update(RobotPose& robotPose)
{
  Pose2f offset = theOdometryData - referenceOdometry;
  robotPose = base + offset;

  MODIFY("module:OdometryOnlySelfLocator:basePose", base);
  DEBUG_RESPONSE_ONCE("module:OdometrOnlySelfLocator:resetReferenceOdometry")
  {
    referenceOdometry = theOdometryData;
  }
}

MAKE_MODULE(OdometryOnlySelfLocator, modeling)
