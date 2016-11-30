
#include "OdometryAnalyzer.h"

MAKE_MODULE(OdometryAnalyzer, modeling)

OdometryAnalyzer::OdometryAnalyzer()
{
}

void OdometryAnalyzer::update(Odometer& odometer)
{
  float difference = (stackingOdometry.translation - firstOdometry.translation).norm();
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
                                        -x * y / 3.0f, x * x / 3.0f, x / 2.0f,
                                        -y / 2.0f, x / 2.0f, 1.0f).finished() * sigmaAngle * sigmaAngle * distance;
  if(difference < 500.f)
  {
    stackingOdometry += odometer.odometryOffset;
    covariance = odometer.odometryOffsetCovariance;
  }
}

void OdometryAnalyzer::update(OdometryAnalyzation& analyzation)
{
  analyzation.startPoint = firstOdometry;
  analyzation.currentEndPoint = stackingOdometry;
  analyzation.covariance << covariance(0, 0), covariance(0, 1), covariance(1, 0), covariance(1, 1);
}
