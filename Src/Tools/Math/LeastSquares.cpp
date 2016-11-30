#include "LeastSquares.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Deviation.h"
#include <limits>

float leastSquaresLineFit(const std::vector<Vector2f>& points, Vector2f& n0, float& d)
{
  // https://de.wikipedia.org/wiki/Lineare_Regression#Berechnung_der_Regressionsgeraden
  Vector2f avg(0.f, 0.f);
  for(const Vector2f& p : points)
  {
    avg += p;
  }
  avg /= static_cast<float>(points.size());

  float SSxx = 0, SSxy = 0;
  for(const Vector2f& p : points)
  {
    const float xDiff = p.x() - avg.x();
    SSxx += sqr(xDiff);
    SSxy += xDiff * (p.y() - avg.y());
  }

  if(Approx::isZero(SSxx))
  {
    // vertical lines cannot be fitted in a y=ax+b style equation
    n0 << 1, 0;
    d = avg.x();
    return 0;
  }

  const float b = SSxy / SSxx;
  const float a = avg.y() - b * avg.x();

  // https://de.wikipedia.org/wiki/Koordinatenform#Koordinatenform_einer_Geradengleichung
  // https://de.wikipedia.org/wiki/Normalenform#Berechnung
  // https://de.wikipedia.org/wiki/Hessesche_Normalform#Berechnung
  const float nLengthNegInv = (float)((a >= 0 ? 1 : -1) / sqrt(sqr(b) + 1));
  n0 << -b* nLengthNegInv, nLengthNegInv;
  d = a * nLengthNegInv;

  float error = 0;
  for(const Vector2f& p : points)
  {
    error += std::abs(n0.dot(p) - d);
  }

  return error / points.size();
}

float leastSquaresCircleFit(const std::vector<Vector2f>& points, Vector2f& center, float& radius)
{
  // Algorithm adapted from http://www.dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
  float xAvg = 0, yAvg = 0;
  float suu = 0, suv = 0, svv = 0, c1 = 0, c2 = 0;

  for(const Vector2f& p : points)
  {
    xAvg += p.x();
    yAvg += p.y();
  }
  xAvg /= points.size();
  yAvg /= points.size();

  for(const Vector2f& p : points)
  {
    const float u = p.x() - xAvg;
    const float v = p.y() - yAvg;

    suu += u * u;
    suv += u * v;
    svv += v * v;
    c1 += u * u * u + u * v * v;
    c2 += v * v * v + v * u * u;
  }
  c1 /= 2;
  c2 /= 2;

  const float divisor = suu * svv - suv * suv;
  if(divisor == 0)
  {
    // No solution -> no circle can be fitted
    return std::numeric_limits<float>::infinity();
  }
  const float uc = (svv * c1 - c2 * suv) / divisor;
  const float vc = (suu * c2 - c1 * suv) / divisor;
  center.x() = uc + xAvg;
  center.y() = vc + yAvg;

  radius = static_cast<float>(sqrt(sqr(uc) + sqr(vc) + (suu + svv) / points.size()));

  float totalError = 0;
  for(const Vector2f& p : points)
  {
    totalError += getRelativeDeviation((p - center).norm(), radius);
  }

  return totalError / points.size();
}
