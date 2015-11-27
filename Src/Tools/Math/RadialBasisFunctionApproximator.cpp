/*
 * File:   RadialBasisFunctionApproximator.cpp
 * Author: arne
 *
 * Created on January 30, 2015, 8:05 PM
 */

#include "RadialBasisFunctionApproximator.h"
#include "Platform/BHAssert.h"

RadialBasisFunctionApproximator::RadialBasisFunctionApproximator(const VectorXf& centers, const float overlap) :
  centers(centers)
{
  ASSERT(centers.cols() > 0);
  initializeWidths(overlap);
}

void RadialBasisFunctionApproximator::initializeWidths(const float overlap)
{
  /* The overlap means that:
   * \f[
   * \phi(c_j) = overlap = e^{-p_{j+1}(c_j - u_{j+1})^2}
   * \f]
   * This can be rearranged to p_{j+i} to get the widths
   */
  widths.resize(centers.rows());
  ASSERT(centers.rows() > 1);
  const float logOverlap = -logf(overlap);
  //for the first gaussian we use the next one as reference
  for(int i = 1; i < widths.size(); ++i)
  {
    const float denom = (centers[i] - centers[i - 1]);
    ASSERT(denom != 0.0f); //avoid div by zero
    widths[i - 1] = logOverlap / (denom * denom);
  }
  widths[centers.rows() - 1] = widths[centers.rows() - 2];
}

float RadialBasisFunctionApproximator::evaluate(const float x)
{
  ASSERT(weightsSet);
  //this is done in array syntax to benefit from eigen's SSE implementation
  phis = (-widths.array() * (x - centers.array()).pow(2)).exp();
  const float nom = x * weights * phis;
  return nom / phis.sum();
}

void RadialBasisFunctionApproximator::setWeights(const RowVectorXf& w)
{
  ASSERT(centers.rows() == w.cols());
  weights = w;
  weightsSet = true;
}

const VectorXf& RadialBasisFunctionApproximator::getWidths() const
{
  return widths;
}

const VectorXf& RadialBasisFunctionApproximator::getCenters() const
{
  return centers;
}
