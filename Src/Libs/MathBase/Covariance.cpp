/**
 * @file Covariance.cpp
 *
 * This file exists to reduce compile time.
 * (A * B.transposed() is an own Eigen type which costs a lot to instantiate)
 */

#include "Covariance.h"

Matrix2f Covariance::rotateCovarianceMatrix(const Matrix2f& covariance, float angle)
{
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  const Matrix2f rotationMatrix = (Matrix2f() << cosine, -sine, sine, cosine).finished();
  return (rotationMatrix * covariance) * rotationMatrix.transpose();
}
