/**
 * @file OdometryOnlySelfLocator.cpp
 *
 * Declares a class that performs self-localization by adding odometry offsets.
 * This is not for real self-localization but for testing and debugging.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "OdometryOnlySelfLocator.h"
#include "Tools/Math/Covariance.h"

void OdometryOnlySelfLocator::update(RobotPose& robotPose)
{
  Pose2f offset = theOdometryData - referenceOdometry;

  // Integrate covariance (using the robot pose prior to update).
  const Matrix2f Q = Eigen::Rotation2D<float>(robotPose.rotation).toRotationMatrix();
  Matrix3f A(Matrix3f::Identity());
  A.topRightCorner<2, 1>() = Q * Vector2f(-theOdometer.odometryOffset.translation.y(), theOdometer.odometryOffset.translation.x());
  Matrix3f B(Matrix3f::Identity());
  B.topLeftCorner<2, 2>() = Q;

  robotPose = base + offset; // This does not touch robot pose attributes not inherited from Pose2D.
  robotPose.covariance = A * robotPose.covariance * A.transpose() + B * theOdometer.odometryOffsetCovariance * B.transpose();
  Covariance::fixCovariance(robotPose.covariance);

  MODIFY("module:OdometryOnlySelfLocator:basePose", base);
  DEBUG_RESPONSE_ONCE("module:OdometryOnlySelfLocator:resetReferenceOdometry")
  {
    referenceOdometry = theOdometryData;
    robotPose.covariance.setIdentity();
    robotPose.covariance(2, 2) = 1e-9f;
  }
}

MAKE_MODULE(OdometryOnlySelfLocator, modeling)
