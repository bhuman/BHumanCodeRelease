#include "CylinderRing.h"

using namespace std;

void CylinderRing::clip(double& minLambda, double& maxLambda, double a, double b)
{
  if(a > 0)
    minLambda = max(minLambda, -b / a); // holds for lambda>=b/a
  else if(a < 0)
    maxLambda = min(maxLambda, -b / a); // holds for lambda<=b/a
  else if(b < 0)  // holds not at all
  {
    minLambda = numeric_limits<double>::infinity();
    maxLambda = -numeric_limits<double>::infinity();
  }
  // else holds always
}

void CylinderRing::clip(double& minLambda, double& maxLambda, double a, double b, double c)
{
  assert(a < 0);
  double p = b / a, q = c / a;
  double rt = p * p / 4 - q;
  if(rt >= 0)  // There is a solution
  {
    double x0, x1;
    if(p > 0)
    {
      x0 = -p / 2 - sqrt(rt);
      x1 = q / x0;
    }
    else
    {
      x1 = -p / 2 + sqrt(rt);
      x0 = q / x1;
    }
    assert(x0 <= x1);
    assert(fabs(x0 + x1 + p) < 1e-3);
    assert(fabs(a * x0 * x0 + b * x0 + c) < 1E-3);
    assert(fabs(a * x1 * x1 + b * x1 + c) < 1E-3);
    minLambda = max(minLambda, x0);
    maxLambda = min(maxLambda, x1);
  }
  else // no solution at all
  {
    minLambda = numeric_limits<double>::infinity();
    maxLambda = -numeric_limits<double>::infinity();
  }
}

void CylinderRing::intersectWithCylinderRay(double& minLambda, double& maxLambda, const Eigen::Vector3d& p, const Eigen::Vector3d& v) const
{
  minLambda = 0;
  maxLambda = numeric_limits<double>::infinity();
  // p(2)+lambda*v(2)>=zLow
  clip(minLambda, maxLambda, v(2), p(2) - zLow);
  // p(2)+lambda*v(2)<=zHigh
  clip(minLambda, maxLambda, -v(2), zHigh - p(2));
  // (p(0..1)+lambda*v(0..1))^2<=rHigh^2
  double a = -(v(0) * v(0) + v(1) * v(1));
  double b = -2 * (p(0) * v(0) + p(1) * v(1));
  double c = -p(0) * p(0) - p(1) * p(1);
  clip(minLambda, maxLambda, a, b, c + rHigh * rHigh);

  // compute part that has to be removed because it is inside the inner cylinder (rLow)
  double minIL = minLambda, maxIL = maxLambda;
  clip(minIL, maxIL, a, b, c + rLow * rLow);

  // remove [minIL..maxIL] from [minLambda..maxLambda] (as far as possible)
  if(minIL <= maxIL)
  {
    if(minIL <= minLambda)
      minLambda = maxIL;
    if(maxIL >= maxLambda)
      maxLambda = minIL;
  }
}

void CylinderRing::intersectWithRaySlow(double& minLambda, double& maxLambda, const Eigen::Vector3d& p, const Eigen::Vector3d& v, double eps) const
{
  double vNormXY   = sqrt(v(0) * v(0) + v(1) * v(1));
  double lambdaEnd = min((rHigh + p.norm()) / vNormXY, max(fabs(p(2) - zLow), fabs(p(2) - zHigh)) / fabs(v(2)));

  minLambda = numeric_limits<double>::infinity();
  maxLambda = -numeric_limits<double>::infinity();
  for(double lambda = 0; lambda <= lambdaEnd; lambda += eps)
  {
    if(isInside(p + lambda * v))
    {
      minLambda = min(minLambda, lambda);
      maxLambda = max(maxLambda, lambda);
    }
  }
}

Eigen::Vector3d CylinderRing::sampleEdgePoint(double angle, int whichEdge) const
{
  Eigen::Vector3d pInCylinder;
  if(whichEdge >= 2)
    pInCylinder(2) = zHigh;
  else
    pInCylinder(2) = zLow;
  double r;
  if(whichEdge % 2 == 0)
    r = rLow;
  else
    r = rHigh;
  pInCylinder(0) = cos(angle) * r;
  pInCylinder(1) = sin(angle) * r;
  return cylinder2World * pInCylinder;
}
