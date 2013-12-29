/**
 * @file Covariance.h
 * @author <a href="mailto:afabisch@tzi.de>Alexander Fabisch</a>
 * Some tools for covariance matrices.
 */
#pragma once

#include "Matrix2x2.h"
#include <cmath>

namespace Covariance
{
/**
 * Creates a covariance of 2 independend random variables.
 * @param dev A vector containing the standard deviations of the random
 *            variables.
 */
inline const Matrix2x2<> create(const Vector2<> dev)
{
  return Matrix2x2<>(Vector2<>(dev.x * dev.x, 0.0f), Vector2<>(0.0f, dev.y * dev.y));
}

/**
 * Creates a covariance of 2 dependend random variables with known rotation.
 * @param dev A vector containing the standard deviations of the random
 *            variables.
 * @param angle The rotation in radians.
 */
inline const Matrix2x2<> create(const Vector2<>& dev, const float angle)
{
  const float sinRotation = std::sin(angle);
  const float cosRotation = std::cos(angle);
  const Matrix2x2<> r = Matrix2x2<>(
                          Vector2<>(cosRotation, sinRotation),
                          Vector2<>(-sinRotation, cosRotation));
  return r * create(dev) * r.transpose();
}

/**
 * Creates a covariance of 2 dependend random variables with known rotation.
 * @param xDev The standard deviations of the random variables in x direction.
 * @param yDev The standard deviations of the random variables in y direction.
 * @param angle The rotation in radians.
 */
inline const Matrix2x2<> create(const float xDev, const float yDev, const float angle)
{
  const float sinRotation = std::sin(angle);
  const float cosRotation = std::cos(angle);
  const Matrix2x2<> r = Matrix2x2<>(
                          Vector2<>(cosRotation, sinRotation),
                          Vector2<>(-sinRotation, cosRotation));
  return r * Matrix2x2<>(Vector2<>(xDev * xDev, 0.0f), Vector2<>(0.0f, yDev * yDev)) * r.transpose();
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
inline void errorEllipse(
  const Matrix2x2<>& covariance,
  float& axis1,
  float& axis2,
  float& angle,
  const float factor = 1.0f)
{
  const float cov012 = covariance.c[0][1] * covariance.c[0][1];
  const float varianceDiff = covariance.c[0][0] - covariance.c[1][1];
  const float varianceDiff2 = varianceDiff * varianceDiff;
  const float varianceSum = covariance.c[0][0] + covariance.c[1][1];
  const float root = std::sqrt(varianceDiff2 + 4.0f * cov012);
  const float eigenValue1 = 0.5f * (varianceSum + root);
  const float eigenValue2 = 0.5f * (varianceSum - root);

  angle = 0.5f * std::atan2(2.0f * covariance.c[0][1], varianceDiff);
  axis1 = 2.0f * std::sqrt(factor * eigenValue1);
  axis2 = 2.0f * std::sqrt(factor * eigenValue2);
}

/**
 * The cholesky decomposition L of a covariance matrix C such that LL^t = C.
 */
inline Matrix2x2<> choleskyDecomposition(const Matrix2x2<>& c)
{
  Matrix2x2<> L;
  L[0][0] = std::sqrt(c[0][0]);
  L[0][1] = c[0][1] / L[0][0];
  L[1][1] = std::sqrt(c[1][1] - L[0][1] * L[0][1]);
  return L;
}

/**
 * The squared Mahalanobis distance between two points a and b. The
 * Mahalanobis distance is the euclidean distance with the components
 * weighted by a covariance matrix.
 */
inline float squaredMahalanobisDistance(const Vector2<>& a, const Matrix2x2<>& c, const Vector2<>& b)
{
  const Vector2<> diff(a - b);
  return diff * (c.invert() * diff);
}
};
