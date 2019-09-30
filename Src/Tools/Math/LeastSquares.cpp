/**
 * @file LeastSquares.cpp
 *
 * Implements functions for fitting models to data points using linear
 * least squares fitting.
 *
 * @author Felix Thielke
 */

#include "LeastSquares.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/BHMath.h"
#include <Eigen/Eigenvalues>

bool LeastSquares::LineFitter::fit(Vector2f& n0, float& d) const
{
  // see R.Duda, P. Hart: Pattern classification and scene analysis. Wiley, 1973. pp 332-335
  ASSERT(count >= 2);

  const auto sum = this->sum.col(0);   // (sum(x), sum(y))
  const auto sum2 = this->sum.col(1);  // (sum(x^2), sum(y^2))
  const float sumXY = this->sum(0, 2); // sum(xy)
  const float nInv = 1.f / static_cast<float>(count);

  // Calculate mean, variance and covariance
  const Vector2f mean(sum * nInv);
  const Vector2f var(sum2 - nInv * sum.cwiseProduct(sum)); // (var_x * count, var_y * count)
  const float covXY = sumXY - nInv * sum.x() * sum.y();    // cov_xy * count

  // Calculate the eigen decomposition of the covariance matrix
  const Eigen::SelfAdjointEigenSolver<Matrix2f> solver(Eigen::SelfAdjointEigenSolver<Matrix2f>().computeDirect((Matrix2f() << var.x(), covXY, covXY, var.y()).finished()));
  if(solver.info() != Eigen::ComputationInfo::Success)
    return false;
  const auto& eigenValues = solver.eigenvalues();

  // n0 is the eigenvector with the smaller eigenvalue
  n0 = solver.eigenvectors().col(eigenValues[0] < eigenValues[1] ? 0 : 1);
  d = n0.dot(mean);

  return true;
}

bool LeastSquares::CircleFitter::fit(Vector2f& center, float& radius) const
{
  // Algorithm adapted from http://www.dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
  ASSERT(count >= 3);

  const auto sum = this->sum.col(0);   // (sum(x), sum(y))
  const auto sum2 = this->sum.col(1);  // (sum(x^2), sum(y^2))
  const auto sum3 = this->sum.col(2);  // (sum(x^3 + x + y^2), sum(y^3 + x^2 + y))
  const float sumXY = this->sum(0, 3); // sum(xy)
  const float nInv = 1.f / static_cast<float>(count);

  // Calculate mean, variance and covariance
  const Vector2f mean(sum * nInv);
  const Vector2f var(sum2 - nInv * sum.cwiseProduct(sum)); // (var_x * count, var_y * count)
  const float covXY = sumXY - nInv * sum.x() * sum.y();    // cov_xy * count

  const float divisor = 2.f * (var.x() * var.y() - sqr(covXY));
  if(Approx::isZero(divisor))
    return false;

  const float coeff1 = 2.f * nInv * sum.squaredNorm();
  const float coeff2 = -2.f * sumXY;
  const Matrix2f m1((Matrix2f() <<
                     (coeff1 - 3.f * sum2.x() - sum2.y()), coeff2,
                     coeff2, (coeff1 - 3.f * sum2.y() - sum2.x())
                    ).finished());

  const Matrix2f m2((Matrix2f() <<
                     var.y(), -covXY,
                     -covXY, var.x()
                    ).finished());

  const Vector2f c(m2 * ((sum3 + m1 * mean) / divisor));

  center = c + mean;
  radius = std::sqrt(c.squaredNorm() + nInv * var.sum());

  return true;
}
