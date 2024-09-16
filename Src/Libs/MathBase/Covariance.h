/**
 * @file Covariance.h
 *
 * Some tools for covariance matrices.
 *
 * @author <a href="mailto:afabisch@tzi.de>Alexander Fabisch</a>
 * @author Tim Laue
 */
#pragma once

#include "MathBase/Eigen.h"
#include <cmath>

namespace Covariance
{
  /**
   * Calculates an ellipse of equiprobable points of a zero centered covariance.
   * This is usually used for debug drawings.
   * @param m The covariance matrix.
   * @param axis1 The major axis of the corresponding ellipse.
   * @param axis2 The minor axis of the corresponding ellipse.
   * @param angle The rotation of the ellipse.
   * @param p Size of the confidence interval covered by the ellipse with 0 < p < 1
   */
  inline void errorEllipse(const Matrix2f& m, float& axis1, float& axis2,
                           float& angle, const float p = 0.95f)
  {
    // Compute the two eigenvalues of the matrix:
    const float trace = m.trace();
    const float rootExpression = std::sqrt(trace * trace - 4.f * m.determinant());
    const float eigenValue1 = 0.5f * (trace + rootExpression);
    const float eigenValue2 = 0.5f * (trace - rootExpression);

    // Axes lengths depend on eigenvalues and the size of the covered confidence.
    // We can consider this whole thing as a Chi-Square distribution with two degrees of freedom:
    float maxEigenValue = eigenValue1;
    float minEigenValue = eigenValue2;
    if(minEigenValue > maxEigenValue)
    {
      maxEigenValue = eigenValue2;
      minEigenValue = eigenValue1;
    }
    const float s = -2.f * std::log10(1.f - p);
    axis1 = 2.0f * std::sqrt(s * maxEigenValue);
    axis2 = 2.0f * std::sqrt(s * minEigenValue);

    // Ellipse angle is the angle of the eigenvector of the larger eigen value:
    angle = 0.f;
    if(m(1,0) != 0.f) // There must be a covariance. Otherwise, the ellipse is not rotated (== axis-aligned)
    {
      // x is set to 1, y is computed by solving the matrix:
      Vector2f eigenVector(1.f, (m(0,0) - maxEigenValue) / -m(1,0));
      angle = std::atan2(eigenVector.y(), eigenVector.x());
    }
  }

  /**
   * Rotate the covariance matrix with given angle.
   * @param covariance The covariance matrix that will be rotated.
   * @param angle The angle of rotation.
   */
  Matrix2f rotateCovarianceMatrix(const Matrix2f& covariance, float angle);

  /**
   * In some covariance matrices, m(i, j) is not equal to m(j, i). This is probably a result
   * of the low precision of "float". This method equals both values.
   * @param m A reference to a matrix that will be changed by this method
   */
  template<int SIZE>
  inline void fixCovariance(Eigen::Matrix<float, SIZE, SIZE>& m)
  {
    for(int i = 0; i < SIZE; i++)
    {
      for(int j = i + 1; j < SIZE; j++)
        m(i, j) = m(j, i) = (m(i, j) + m(j, i)) * 0.5f;
    }
  }
}
