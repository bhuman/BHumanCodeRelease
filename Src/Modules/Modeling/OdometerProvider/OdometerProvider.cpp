/*
 * @file OdometerProvider.cpp
 *
 * Implementation of module that computes some additional odometry information
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 * @author marcel
 */

#include "OdometerProvider.h"

MAKE_MODULE(OdometerProvider, modeling)

void OdometerProvider::update(Odometer& odometer)
{
  odometer.odometryOffset = theOdometryData - lastOdometryData;
  odometer.distanceWalked += odometer.odometryOffset.translation.norm();
  lastOdometryData = theOdometryData;
}
