/**
 * @file OrientationKalmanFilter.h
 *
 * A simple 1D-Kalman-Filter implementation which correctly interpolate angles in the range of -180 to 180 degree.
 *
 * @author <a href="mailto:muehlenb@tzi.de">Andre Muehlenbrock</a>
 */

#include "OrientationKalmanFilter.h"
#include "Tools/Debugging/DebugDrawings.h"

float norm(float deg)
{
  while(deg < 0)
    deg += 2 * pi;

  while(deg >= 2 * pi)
    deg -= 2 * pi;

  return deg;
}

OrientationKalmanFilter::OrientationKalmanFilter(const float pEstimatedError, const float pInitialValue)
{
  covariance = pEstimatedError;
  mean = pInitialValue;
}

void OrientationKalmanFilter::predict(const float pDynamicNoise)
{
  covariance += pDynamicNoise;
}

void OrientationKalmanFilter::rotate(float angle)
{
  mean += angle;
  norm(mean);
}

void OrientationKalmanFilter::update(const float measurement, const float pMeasureNoise)
{
  gain = covariance / (covariance + pMeasureNoise);

  // is the measured orientation in higher direction?
  bool upwards = (measurement - mean > 0 && measurement - mean < pi) || (measurement + 2 * pi - mean > 0 && measurement + 2 * pi - mean < pi);

  float distance = std::min(std::abs(measurement - mean),
                            std::abs((measurement > mean ? measurement - 2 * pi : mean - 2 * pi) - (measurement > mean ? mean : measurement)));

  // turn in correct direction (caution: if state = 178 and measurement = -175, the distance has to be ADDED and not subtracted!)
  mean += gain * (upwards ? distance : -distance);
  mean = norm(mean);

  covariance = (1 - gain) * covariance;
}

float OrientationKalmanFilter::getMean()
{
  return mean;
}

float OrientationKalmanFilter::getCovariance()
{
  return covariance;
}