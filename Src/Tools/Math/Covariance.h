/**
 * @file Covariance.h
 * @author <a href="mailto:afabisch@tzi.de>Alexander Fabisch</a>
 * Some tools for covariance matrices.
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
   * @param covariance The covariance matrix.
   * @param axis1 The major axis of the corresponding ellipse.
   * @param axis2 The minor axis of the corresponding ellipse.
   * @param angle The rotation of the ellipse.
   * @param factor A scaling factor for the axes.
   */
  inline void errorEllipse(const Matrix2f& covariance, float& axis1, float& axis2,
                           float& angle, const float factor = 1.0f)
  {
    const float cov012 = covariance(1, 0) * covariance(1, 0);
    const float varianceDiff = covariance(0, 0) - covariance(1, 1);
    const float varianceDiff2 = varianceDiff * varianceDiff;
    const float varianceSum = covariance(0, 0) + covariance(1, 1);
    const float root = std::sqrt(varianceDiff2 + 4.0f * cov012);
    const float eigenValue1 = 0.5f * (varianceSum + root);
    const float eigenValue2 = 0.5f * (varianceSum - root);

    angle = 0.5f * std::atan2(2.0f * covariance(1, 0), varianceDiff);
    axis1 = 2.0f * std::sqrt(factor * eigenValue1);
    axis2 = 2.0f * std::sqrt(factor * eigenValue2);
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
};
