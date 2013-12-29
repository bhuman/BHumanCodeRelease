/**
* @file GaussianDistribution2D.cpp
*
* Implementation of class GaussianDistribution2D
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "Probabilistics.h"
#include "GaussianDistribution2D.h"


float GaussianDistribution2D::distanceTo(const GaussianDistribution2D& other) const
{
  Vector2<> diff(mean - other.mean);
  Matrix2x2<> cov(covariance + other.covariance);
  return diff * (cov.invert() * diff);
}

float GaussianDistribution2D::probabilityAt(const Vector2<>& pos) const
{
  Vector2<> diff(pos - mean);
  float exponent(diff * (covariance.invert()*diff));
  float probability(1.0f / (pi2 * std::sqrt(covariance.det())));
  probability *= exp(-0.5f * exponent);
  return std::max(probability, 0.000001f);
}

void GaussianDistribution2D::generateDistributionFromMeasurements(
  float* x, int numOfX, float* y, int numOfY)
{
  if(numOfX < 2 || numOfY < 2)
    return;
  mean.x = mean.y = 0.0f;
  for(int i = 0; i < numOfX; i++)
    mean.x += x[i];
  for(int i = 0; i < numOfY; i++)
    mean.y += y[i];
  mean.x /= numOfX;
  mean.y /= numOfY;
  float varianceX(0.0f);
  float varianceY(0.0f);
  for(int i = 0; i < numOfX; i++)
    varianceX += (x[i] - mean.x) * (x[i] - mean.x);
  varianceX *= 1.0f / (numOfX - 1);
  for(int i = 0; i < numOfY; i++)
    varianceY += (y[i] - mean.y) * (y[i] - mean.y);
  varianceY *= 1.0f / (numOfY - 1);
  covariance[0][0] = varianceX;
  covariance[1][1] = varianceY;
  float cov_xy(0.0f);
  int maxXY = numOfX > numOfY ? numOfY : numOfX;
  for(int i = 0; i < maxXY; i++)
    cov_xy += (x[i] - mean.x) * (y[i] - mean.y);
  cov_xy *= 1.0f / (maxXY - 1);
  covariance[0][1] = covariance[1][0] = cov_xy;
}

GaussianDistribution2D& GaussianDistribution2D::operator +=(const GaussianDistribution2D& other)
{
  Matrix2x2<float> K(covariance);
  K *= (covariance + other.covariance).invert();
  mean += K * (other.mean - mean);
  covariance -= K * covariance;
  return *this;
}

void GaussianDistribution2D::merge(const GaussianDistribution2D& other)
{
  //Compute new mean and values for new covariance matrix:
  float p1(probabilityAtMean());
  float p2(other.probabilityAtMean());
  float pSum(p1 + p2);
  float w1(p1 / pSum);
  float w2(p2 / pSum);
  Vector2<> newMean(mean * w1 + other.mean * w2);
  Vector2<> diffMean(mean - other.mean);
  Matrix2x2<float> meanMatrix;
  meanMatrix.c[0].x = diffMean.x * diffMean.x;
  meanMatrix.c[0].y = meanMatrix.c[1].x = diffMean.x * diffMean.y;
  meanMatrix.c[1].y = diffMean.y * diffMean.y;
  //Set new values:
  mean = newMean;
  covariance = (covariance * w1 + other.covariance * w2 + meanMatrix * w1 * w2);
}

Vector2<> GaussianDistribution2D::rand() const
{
  Matrix2x2<float> L;
  choleskyDecomposition(covariance, L, 1E-9f * (covariance.trace()));
  Vector2<> xRaw(randomGauss(), randomGauss());
  return L * xRaw;
}

bool GaussianDistribution2D::choleskyDecomposition(const Matrix2x2<float>& A,
                                                   Matrix2x2<float>& L, float eps) const
{
  float sum(A.c[0].x);
  if(sum < -eps)
    return false;
  if(sum > 0)
  {
    // (0,0)
    L[0][0] = std::sqrt(sum);
    // (1,0)
    sum = A.c[1].x;
    L[1][0] = sum / L[0][0];
  }
  else
    L[0][0] = L[1][0] = 0;

  // (0,1)
  L[0][1] = 0;
  // (1,1)
  sum = A.c[1].y - L[1][0] * L[1][0];
  if(sum < -eps)
    return false;
  if(sum > 0)
    L[1][1] = std::sqrt(sum);
  else
    L[1][1] = 0;

  return true;
}

void GaussianDistribution2D::getEigenVectorsAndEigenValues(Vector2<>& eVec1, Vector2<>& eVec2,
                                                           float& eValue1, float& eValue2) const
{
  // Compute eigenvalues
  float cmTrace(covariance.trace());
  float cmDet(covariance.det());
  float sqrtExpression(std::sqrt(cmTrace * cmTrace - 4 * cmDet));
  eValue1 = 0.5f * (cmTrace + sqrtExpression);
  eValue2 = 0.5f * (cmTrace - sqrtExpression);
  // Compute eigenvectors, general fomula:
  // | cm.c[0].x - eValue  cm.c[1].x          |  *  |x|   = |0|
  // | cm.c[0].y           cm.c[1].y - eValue |     |y|     |0|
  // First eigenvector (x set to 1):
  eVec1.x = 1.0f;
  eVec1.y = -covariance.c[0].y / (covariance.c[1].y - eValue1);
  eVec1.normalize();
  // Second eigenvector (y set to 1):
  eVec2.x = -covariance.c[1].x / (covariance.c[0].x - eValue2);
  eVec2.y = 1.0f;
  eVec2.normalize();
}
