/**
 * @file Covariance.h
 *
 * Some tools for covariance matrices.
 *
 * @author <a href="mailto:afabisch@tzi.de>Alexander Fabisch</a>
 * @author Tim Laue
 */
#pragma once

#include "Eigen.h"

namespace Covariance
{
  /**
   * Creates a covariance of 2 independend random variables.
   * @param dev A vector containing the standard deviations of the random
   *            variables.
   */
  inline const Matrix2f create(const Vector2f& dev)
  {
    return dev.array().square().matrix().asDiagonal();
  }

  /**
   * Creates a covariance of 2 dependend random variables with known rotation.
   * @param dev A vector containing the standard deviations of the random
   *            variables.
   * @param angle The rotation in radians.
   */
  inline const Matrix2f create(const Vector2f& dev, const float angle)
  {
    const float sinRotation = std::sin(angle);
    const float cosRotation = std::cos(angle);
    const Matrix2f r = (Matrix2f() << cosRotation, -sinRotation,
                                      sinRotation, cosRotation).finished();
    return r * create(dev) * r.transpose();
  }

  /**
   * Creates a covariance of 2 dependend random variables with known rotation.
   * @param xDev The standard deviations of the random variables in x direction.
   * @param yDev The standard deviations of the random variables in y direction.
   * @param angle The rotation in radians.
   */
  inline const Matrix2f create(const float xDev, const float yDev, const float angle)
  {
    const float sinRotation = std::sin(angle);
    const float cosRotation = std::cos(angle);
    const Matrix2f r = (Matrix2f() << cosRotation, -sinRotation,
                                      sinRotation, cosRotation).finished();
    return r * (Matrix2f() << xDev * xDev, 0.0f, 0.0f, yDev * yDev).finished() * r.transpose();
  }

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
   * The cholesky decomposition L of a covariance matrix C such that LL^t = C.
   */
  inline Matrix2f choleskyDecomposition(const Matrix2f& c)
  {
    Eigen::LLT<Matrix2f> llt = c.llt();
    ASSERT(llt.info() == Eigen::ComputationInfo::Success);
    return llt.matrixL();
  }

  /**
   * The squared Mahalanobis distance between two points a and b. The
   * Mahalanobis distance is the euclidean distance with the components
   * weighted by a covariance matrix.
   */
  inline float squaredMahalanobisDistance(const Vector2f& a, const Matrix2f& c, const Vector2f& b)
  {
    const Vector2f diff = a - b;
    return diff.dot(c.inverse() * diff);
  }

  /**
   * Rotate the covariance matrix with given angle.
   * @param covariance The covariance matrix that will be rotated.
   * @param angle The angle of rotation.
   */
  inline Matrix2f rotateCovarianceMatrix(const Matrix2f& covariance, float angle)
  {
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    const Matrix2f rotationMatrix = (Matrix2f() << cosine, -sine, sine, cosine).finished();
    return (rotationMatrix * covariance) * rotationMatrix.transpose();
  }

  /**
   * In some covariance matrices, m(0, 1) is not equal to m(1, 0). This is probably a result
   * of the low precision of "float". This method equals both values.
   * @param m A reference to a matrix that will be changed by this method
   */
  inline void fixCovariance(Matrix2f& m)
  {
    m(0, 1) = m(1, 0) = (m(0, 1) + m(1, 0)) * .5f;
  }

  /**
   * In some covariance matrices, m(0, 1) is not equal to m(1, 0) and so on. This is probably a result
   * of the low precision of "float". This method equals both values.
   * @param m A reference to a matrix that will be changed by this method
   */
  inline void fixCovariance(Matrix3f& m)
  {
    m(0, 1) = m(1, 0) = (m(0, 1) + m(1, 0)) * .5f;
    m(1, 2) = m(2, 1) = (m(1, 2) + m(2, 1)) * .5f;
    m(0, 2) = m(2, 0) = (m(0, 2) + m(2, 0)) * .5f;
  }
};
