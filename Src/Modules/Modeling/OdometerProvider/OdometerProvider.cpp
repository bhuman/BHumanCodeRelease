/*
 * @file OdometerProvider.cpp
 *
 * Implementation of module that computes some additional odometry information
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author marcel
 */

#include "OdometerProvider.h"

MAKE_MODULE(OdometerProvider);

void OdometerProvider::update(Odometer& odometer)
{
  odometer.odometryOffset = theOdometryData - lastOdometryData;
  const float distance = odometer.odometryOffset.translation.norm();
  odometer.distanceWalked += distance;
  lastOdometryData = theOdometryData;

  odometer.odometryOffsetCovariance.setZero();
  odometer.odometryOffsetCovariance(0, 0) = sigmaDistance * sigmaDistance * distance;
  odometer.odometryOffsetCovariance(1, 1) = sigmaDistance * sigmaDistance * distance;
  const float x = odometer.odometryOffset.translation.x();
  const float y = odometer.odometryOffset.translation.y();
  odometer.odometryOffsetCovariance += (Matrix3f() << y * y / 3.0f, -x * y / 3.0f, -y / 2.0f,
                                        -x * y / 3.0f,  x * x / 3.0f,  x / 2.0f,
                                        -y / 2.0f,    x / 2.0f,  1.0f).finished() * sigmaAngle * sigmaAngle * distance;
}
