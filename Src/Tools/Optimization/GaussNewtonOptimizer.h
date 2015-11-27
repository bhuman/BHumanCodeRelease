#pragma once

#include <vector>
#include "Tools/Debugging/Debugging.h"
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include <Eigen/Core>
#include <Eigen/LU>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

/**
 * This class implements the Gauss-Newton algorithm.
 * A set of parameters is optimized in regard of the sum of squared errors using
 * a given error function. The jacobian that is computed in each iteration is
 * approximated numerically.
 * @tparam M The class that represents a single measurement / sample.
 * @tparam C The class the error function is a member of.
 */
template <class M, class C>
class GaussNewtonOptimizer
{
private:
  using Vector = Eigen::Matrix<float, Eigen::Dynamic, 1>;
  using Matrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

  const unsigned int numOfMeasurements; /**< The number of measurements. */
  std::vector<float> currentParameters; /**< The vector (Nx1-matrix) containing the current parameters. */
  Vector currentValues; /**< The vector (Nx1-matrix) containing the current error values for all measurements. */
  const std::vector<M>& measurements; /**< A reference to the vector containing all measurements. */
  const C& object; /**< The object used to call the error function. */
  float(C::*pFunction)(const M& measurement, const std::vector<float>& parameters) const;  /**< A pointer to the error function. */
  const float delta = 0.001f; /**< The delta used to approximate the partial derivatives of the Jacobian. */

public:
  GaussNewtonOptimizer(const std::vector<float>& parameters, const std::vector<M>& measurements, const C& object,
                       float(C::*pFunction)(const M& measurement, const std::vector<float>& parameters) const) :
    numOfMeasurements(static_cast<unsigned>(measurements.size())), currentParameters(parameters),
    currentValues(numOfMeasurements, 1), measurements(measurements), object(object), pFunction(pFunction)
  {
    for(unsigned int i = 0; i < numOfMeasurements; ++i)
      currentValues(i, 0) = (object.*pFunction)(measurements[i], currentParameters);
  }

  /**
   * This method executes one iteration of the Gauss-Newton algorithm.
   * The new parameter vector is computed by a_i+1 = a_i - (D^T * D)^-1 * D^T * r
   * where D is the Jacobian, a is the parameter vector and r is the vector containing the current error values.
   * @return The sum of absolute differences between the old and the new parameter vector.
   */
  float iterate()
  {
    // build jacobi matrix
    Matrix jacobiMatrix(numOfMeasurements, currentParameters.size());
    for(unsigned int j = 0; j < currentParameters.size(); ++j)
    {
      // the first derivative is approximated using values slightly above and below the current value
      const float oldParameter = currentParameters[j];
      const float parameterAbove = oldParameter + delta;
      const float parameterBelow = oldParameter - delta;
      for(unsigned int i = 0; i < numOfMeasurements; ++i)
      {
        // approximate first derivation numerically
        currentParameters[j] = parameterAbove;
        const float valueAbove = (object.*pFunction)(measurements[i], currentParameters);
        currentParameters[j] = parameterBelow;
        const float valueBelow = (object.*pFunction)(measurements[i], currentParameters);
        const float derivation = (valueAbove - valueBelow) / (2.0f * delta);
        jacobiMatrix(i, j) = derivation;
      }
      currentParameters[j] = oldParameter;
    }

    try
    {
      Vector result = (jacobiMatrix.transpose() * jacobiMatrix).inverse() * jacobiMatrix.transpose() * currentValues;
      for(unsigned int i = 0; i < currentParameters.size(); ++i)
        currentParameters[i] -= result(i);

      for(unsigned int i = 0; i < numOfMeasurements; ++i)
        currentValues(i, 0) = (object.*pFunction)(measurements[i], currentParameters);

      float sum = 0;
      for(unsigned int i = 0; i < currentParameters.size(); ++i)
        sum += std::abs(result(i));

      return sum;
    }
    catch(...)
    {
      OUTPUT_WARNING("YELP! Catching Exception!");
      return 0.0f;
    }
  }

  /**
   * The method returns the current parameter vector.
   * @return The current parameter vector.
   */
  const std::vector<float>& getParameters() const
  {
    return currentParameters;
  }
};
