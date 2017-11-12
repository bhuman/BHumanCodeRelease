#include "LeastSquares.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Deviation.h"
#include <limits>

void leastSquaresLineFit(const std::vector<Vector2f>& points, Vector2f& n0, float& d)
{
  // https://de.wikipedia.org/wiki/Lineare_Regression#Berechnung_der_Regressionsgeraden
  ASSERT(points.size() >= 2);
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
  }
  else
  {
    const float b = SSxy / SSxx;
    const float a = avg.y() - b * avg.x();

    // https://de.wikipedia.org/wiki/Koordinatenform#Koordinatenform_einer_Geradengleichung
    // https://de.wikipedia.org/wiki/Normalenform#Berechnung
    // https://de.wikipedia.org/wiki/Hessesche_Normalform#Berechnung
    const float nLengthNegInv = (float)((a >= 0 ? 1 : -1) / sqrt(sqr(b) + 1));
    n0 << -b* nLengthNegInv, nLengthNegInv;
    d = a * nLengthNegInv;
  }
}

float leastSquaresLineFitWithError(const std::vector<Vector2f>& points, Vector2f& n0, float& d)
{
  leastSquaresLineFit(points, n0, d);

  float error = 0;
  for(const Vector2f& p : points)
    error += getAbsoluteDeviation(n0.dot(p), d);

  return error / points.size();
}

void leastSquaresCircleFit(const std::vector<Vector2f>& points, Vector2f& center, float& radius)
{
  // Algorithm adapted from http://www.dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
  ASSERT(points.size() >= 3);
  Vector2f avg(0, 0);

  for(const Vector2f& p : points)
  {
    avg += p;
  }
  avg /= static_cast<float>(points.size());

  float suu = 0, suv = 0, svv = 0, c1 = 0, c2 = 0;

  for(const Vector2f& p : points)
  {
    const Vector2f normPoint(p - avg);

    suu += normPoint.x() * normPoint.x();
    suv += normPoint.x() * normPoint.y();
    svv += normPoint.y() * normPoint.y();
    c1 += normPoint.x() * normPoint.x() * normPoint.x() + normPoint.x() * normPoint.y() * normPoint.y();
    c2 += normPoint.y() * normPoint.y() * normPoint.y() + normPoint.y() * normPoint.x() * normPoint.x();
  }
  c1 /= 2;
  c2 /= 2;

  const float divisor = suu * svv - suv * suv;
  if(divisor == 0)
  {
    // No solution -> no circle can be fitted
    radius = std::numeric_limits<float>::infinity();
    return;
  }
  const Vector2f c = Vector2f(svv * c1 - c2 * suv, suu * c2 - c1 * suv) / divisor;
  center = c + avg;

  radius = static_cast<float>(sqrt(c.squaredNorm() + (suu + svv) / static_cast<float>(points.size())));
}
