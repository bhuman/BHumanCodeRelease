/**
 * @file OrientationKalmanFilter.h
 *
 * A simple 1D-Kalman-Filter which correctly interpolate angles in the range of -180 to 180 degree.
 *
 * @author <a href="mailto:muehlenb@tzi.de">Andre Muehlenbrock</a>
 */

#include <cmath>
#include <algorithm>

class OrientationKalmanFilter
{
public:
  OrientationKalmanFilter(const float covariance, const float mean);
  void predict(const float pDynamicNoise);
  void update(const float measurement, const float pMeasureNoise);
  float getMean();
  float getCovariance();
  void rotate(float angle);

private:
  float covariance = 0;
  float mean = 0;
  float gain = 0;
};
