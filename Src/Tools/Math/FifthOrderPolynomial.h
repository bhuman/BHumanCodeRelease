/**
 * File:   FifthOrderPolynomial.h
 * Author: arne
 *
 * Created on January 28, 2015, 3:32 PM
 */

#pragma once

#include "Eigen.h"

/**
 * A 5'th order polynomial defined by 6 boundary conditions.
 * The boundary conditions are two points on the polynomial as well as their
 * first and second derivatives.
 */
class FifthOrderPolynomial
{
private:
  Vector6f c = Vector6f::Zero(); /**< The coefficients of the polynomial */

public:
  FifthOrderPolynomial() = default;

  /**
   * @param x0 the first x value
   * @param fx0 f(x0)
   * @param fx0d f'(x0)
   * @param fx0dd f''(x0)
   * @param x1 the second x value
   * @param fx1 f(x1)
   * @param fx1d f'(x1)
   * @param fx1dd f''(x1)
   */
  FifthOrderPolynomial(const float x0, const float fx0, const float fx0d, const float fx0dd,
                       const float x1, const float fx1, const float fx1d, const float fx1dd);

  /**
   * Evaluates the polynomial at the specified x.
   * @returns [f(x), f'(x), f''(x)]^T
   */
  Vector3f evaluate(const float x) const;
};
