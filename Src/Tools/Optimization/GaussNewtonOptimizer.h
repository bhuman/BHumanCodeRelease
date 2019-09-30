#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/Eigen.h"
#include <Eigen/Cholesky>

template<size_t N>
class GaussNewtonOptimizer
{
public:
  using Vector = Eigen::Matrix<float, N, 1>;

  struct Functor
  {
    /**
     * Calculate the error for the ith measurement given a set of parameters.
     * @parameter measurement The index of the ith measurement.
     */
    virtual float operator()(const Vector& params, size_t measurement) const = 0;

    /** The number of measurements. */
    virtual size_t getNumOfMeasurements() const = 0;
  };

private:
  using Jacobian = Eigen::Matrix<float, Eigen::Dynamic, N>;
  using JacobianTransposed = Eigen::Matrix<float, N, Eigen::Dynamic>;

  const Functor& functor;

public:
  GaussNewtonOptimizer(const Functor& functor) : functor(functor) {};

  float iterate(Vector& params, const Vector& epsilon);
};

template<size_t N>
float GaussNewtonOptimizer<N>::iterate(Vector& params, const Vector& epsilon)
{
  // See: https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm

  const size_t numOfMeasurements = functor.getNumOfMeasurements();
  ASSERT(numOfMeasurements >= N);

  // build jacobi matrix
  Jacobian J(numOfMeasurements, N);
  for(size_t j = 0; j < N; ++j)
  {
    Vector epsilonj = Vector::Zero();
    epsilonj(j) = epsilon(j);
    const Vector paramsAbove = params + epsilonj;
    const Vector paramsBelow = params - epsilonj;
    for(size_t i = 0; i < numOfMeasurements; ++i)
      J(i, j) = (functor(paramsAbove, i) - functor(paramsBelow, i)) / (2 * epsilon(j));
  }

  VectorXf r = VectorXf(numOfMeasurements);
  for(size_t i = 0; i < numOfMeasurements; ++i)
    r(i) = functor(params, i);

  const JacobianTransposed Jt = J.transpose();
  const Eigen::LDLT<Eigen::Matrix<float, N, N>> JtJdecomposed(Jt * J);
  const Vector s = JtJdecomposed.solve(Jt * r);

  params -= s;

  float sum = 0.f;
  for(size_t i = 0; i < N; ++i)
    sum += std::abs(s(i));

  return sum;
}
