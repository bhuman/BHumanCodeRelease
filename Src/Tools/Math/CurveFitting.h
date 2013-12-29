/**
* @file CurveFitting.h
* Fit a polynomial to a series of values.
* @author Felix Wenk
*/

#pragma once

#include "Platform/BHAssert.h"
#include "YaMatrix.h"
#include "Vector2.h"
#include <vector>

template<class T>
T pow(T base, unsigned int exponent)
{
  T result = T(1);
  for(unsigned int i = 0; i < exponent; i++)
    result = result * base;
  return result;
}

/**
 * Function to fit a polynomial of degree m-1 to a series of n values.
 *
 * Tries to calculate coefficients for the polynomial p such that
 * p(points[i][0]) equals points[i][1].
 */
template <class V>
YaMatrix<V> fitLinear(const std::vector<Vector2<V> >& points, const YaMatrix<V>& covariance, unsigned int degree = 2)
{
  int numberOfSamples = points.size();
  ASSERT(numberOfSamples == covariance.M && numberOfSamples == covariance.N);

  YaMatrix<V> A(numberOfSamples, degree + 1);
  YaMatrix<V> values(numberOfSamples, 1);
  for(int i = 0; i < numberOfSamples; i++)
  {
    values[i][0] = points[i].y;
    for(unsigned int d = 0; d <= degree; d++)
      A[i][d] = pow(points[i].x, d);
  }

  const YaMatrix<V> ATransposed = A.transpose();
  const YaMatrix<V> covarianceInv = covariance.inverse();

  return (ATransposed * covarianceInv * A).inverse() * ATransposed * covarianceInv * values;
}
