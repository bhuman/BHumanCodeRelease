/**
 * File:   RadialBasisFunctionApproximator.h
 * Author: arne
 *
 * Created on January 30, 2015, 8:05 PM
 */

#pragma once
#include <vector>
#include "Tools/Math/Eigen.h"

/**
 * A 1 dimensional gaussian radial basis function approximator.
 * It can be used to approximate any non linear function.
 *
 * calling evaluate(x) solves the following equation:
 * \f[
 *    f(x) = \frac{\sum_{j=1}^Nw_j\phi_j(x)x}{\sum_{j=1}^N\phi(x)}
 * \f]
 * where phi is:
 * \f[
 *    \phi(x) = \phi(x)=e^{-p_j(x - u_j)^2}
 * \f]
 * N is the number of radial basis functions, p_j is the width of the j'th
 * radial basis function, w_j is the weight of the j'th function and u_j is the
 * center (i.e. phi(u_j)=1) of the j'th function.
 *
 * The weights can be learned using linear least squares
 * because the approximating function is linear in the weights.
 *
 */
class RadialBasisFunctionApproximator
{
private:
  VectorXf centers;
  RowVectorXf weights;
  VectorXf widths;
  VectorXf phis; /**<temporary used inside evaluate() */
  bool weightsSet = false;

public:
  RadialBasisFunctionApproximator() = default;

  /**
   * @param centers Positions of the centers of the radial basis functions
   * @param overlap Overlap between the radial basis functions. The width of each
   *                gauss function will be chosen in a way that it overlaps by
   *                this value with it's predecessor. I.e.
   *                \f[
   *                    \phi_{j+1}(c_j) = overlap
   *                \f]
   *                c_j is the center of the j'th gaussian.
   * @param weights The weights of each radial basis function.
   * @return
   */
  RadialBasisFunctionApproximator(const VectorXf& centers, const float overlap);

  void setWeights(const RowVectorXf& weights);

  /** evaluate the function at x */
  float evaluate(const float x);

  const VectorXf& getWidths() const;

  const VectorXf& getCenters() const;

private:
  /**
   * Calculates the widths according to overlap.
   */
  void initializeWidths(const float overlap);
};
