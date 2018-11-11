/**
 * @file Gaussian.h
 *
 * This file declares (and implements) a class that represents a gaussian distribution.
 *
 * @author Jonas Kuball <jkuball@tzi.de>
 */

#pragma once

#include "Random.h"
#include "Eigen.h"

#include <limits>

namespace impl
{
  // Static members for uniform distributed values.
  static std::mt19937& rnd = Random::getGenerator();
  static const unsigned int rand_max = std::mt19937::max();

  /**
   * @class SingleDimGaussian
   *
   * This class implements a simple Gaussian Distribution. It is possible to get samples via the call operator.
   * Sampling from an object of this class is faster than std::normal_distribution.
   * Using other Scalar types than floating point types is pretty much undefined, but might work.
   */
  template<typename Scalar>
  class SingleDimGaussian
  {
  public:
    // Internal types to distinguish different things.
    using MeanType = Scalar;
    using StdDevType = Scalar;
    using VarianceType = Scalar;

    /**
     * Default constructor. Constructs the standard Gaussian with mean 0 and a variance of 1.
     */
    SingleDimGaussian() : theMean(0), theStdDev(1), theVariance(1) {};

    /**
     * Parameter constructor. Please note that this takes the mean and the standard deviation and not the mean and the variance!
     */
    SingleDimGaussian(const MeanType mean, const StdDevType stddev) : theMean(mean), theStdDev(stddev), theVariance(stddev * stddev) {};

    /**
     * Copy assignment operator.
     */
    SingleDimGaussian& operator=(const SingleDimGaussian& other)
    {
      theMean = other.theMean;
      theStdDev = other.theStdDev;
      theVariance = other.theVariance;
      return *this;
    }

    /**
     * Copy constructor.
     */
    SingleDimGaussian(const SingleDimGaussian& other) : theMean(other.mean()), theStdDev(other.stddev()), theVariance(other.variance()) {};

    /**
     * Call operator. Returns a random sample from this distribution.
     */
    MeanType operator()();

    /**
     * Calculates the probability density of this gaussian at the given point.
     */
    float pdf(const MeanType& point) const;

    /**
     * Read attributes.
     */
    MeanType mean() const { return theMean; };
    StdDevType stddev() const { return theStdDev; };
    VarianceType variance() const { return theVariance; };

    /**
     * Encapsulate writing of attributes, because stddev and variance are dependent on each other.
     */
    void setMean(MeanType mean) { theMean = mean; };
    void setStdDev(StdDevType stddev) { theStdDev = stddev; theVariance = stddev * stddev; };
    void setVariance(VarianceType variance) { theStdDev = std::sqrt(variance); theVariance = variance; };

  private:
    MeanType theMean;
    StdDevType theStdDev;
    VarianceType theVariance;
    bool generate = false; // since we're generating two samples at once, we only need to calculate them every second time.
    const MeanType epsilon = std::numeric_limits<Scalar>::min();
    MeanType z0, z1;
  };

  /**
   * Implementation of the call operator.
   * The scalar is sampled via Box-Muller transform which samples two normal distributed values at a time.
   */
  template<typename Scalar>
  typename SingleDimGaussian<Scalar>::MeanType SingleDimGaussian<Scalar>::operator()()
  {
    generate = !generate;
    if(!generate)
      return z1 * theStdDev + theMean;

    Scalar u1, u2;
    do
    {
      u1 = rnd() * (1.0f / rand_max);
      u2 = rnd() * (1.0f / rand_max);
    }
    while(u1 <= epsilon);

    z0 = static_cast<Scalar>(sqrt(-2 * log(u1)) * cos(pi2 * u2));
    z1 = static_cast<Scalar>(sqrt(-2 * log(u1)) * sin(pi2 * u2));

    return z0 * theStdDev + theMean;
  }

  /**
   * Implementation of the single dimensional probability density function.
   */
  template<typename Scalar>
  float SingleDimGaussian<Scalar>::pdf(const MeanType& point) const
  {
    return 1.f / std::sqrt(pi2 * theVariance) * std::exp(-sqr(((point - theMean) / theStdDev)));
  }

  /**
   * @struct ScalarNormalDistOp
   * Helper struct used by Eigen to generate matrices filled with normal distributed vaules.
   */
  template<typename Scalar>
  struct ScalarNormalDistOp
  {
    mutable SingleDimGaussian<Scalar> norm;

    template<typename Index>
    inline const Scalar operator()(Index, Index = 0) const { return norm(); }
  };

  /**
   * @class MultivariateGaussian
   *
   * This class implements a Multivariate Gaussian Distribution. It is possible to get samples via the call operator.
   */
  template<typename Scalar, int N>
  class MultivariateGaussian
  {
  public:
    // Internal types to distinguish different things.
    using MeanType = Eigen::Matrix<Scalar, N, 1>;
    using MeansType = Eigen::Matrix<Scalar, N, Eigen::Dynamic>;
    using CovarianceType = Eigen::Matrix<Scalar, N, N>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Default constructor. Constructs the standard Gaussian with mean 0 and a variance of 1.
     */
    MultivariateGaussian() : theMean(MeanType::Zero()), theCovariance(CovarianceType::Identity()) {};

    /**
     * Parameter constructor. Please note that this takes the mean and the standard deviation and not the mean and the variance!
     */
    MultivariateGaussian(const MeanType mean, const CovarianceType covariance) : theMean(mean), theCovariance(covariance) {};

    /**
     * Copy constructor.
     */
    MultivariateGaussian(const MultivariateGaussian& other) : theMean(other.mean()), theCovariance(other.covariance()) {};

    /**
     * Copy assignment operator.
     */
    MultivariateGaussian& operator=(const MultivariateGaussian& other)
    {
      theMean = other.theMean;
      theCovariance = other.theCovariance;
      if(other.solved)
      {
        transform = other.transform;
      }
      solved = other.solved;
      return *this;
    }

    /**
     * Call operator. Returns a list of random samples from this distribution.
     */
    MeansType operator()(const unsigned numOfSamples = 1);

    /**
     * Calculates the probability density of this gaussian at the given point.
     */
    float pdf(const MeanType& point) const;

    /**
     * Forces solving of the covariance matrix, if needed.
     * You dont't need to call this manually, this will be called the first time you retreive a sample.
     * In case you know you're sampling from this distribution in the near future, you could solve it before.
     */
    void solve();

    /**
     * Read attributes.
     */
    MeanType mean() const { return theMean; };
    CovarianceType covariance() const { return theCovariance; };
    bool isSolved() const { return solved; };

    /**
     * Encapsulate writing of attributes, because solved and transform are dependent on the covariance.
     */
    void setMean(MeanType mean) { theMean = mean; };
    void setCovariance(CovarianceType covariance) { theCovariance = covariance; solved = false; transform = CovarianceType::Zero(); };

  private:
    MeanType theMean;
    CovarianceType theCovariance;
    bool solved = false; // To be able to sample from this distribution, we need to solve the covariance matrix once.
    CovarianceType transform;
    ScalarNormalDistOp<Scalar> normal;
  };

  /**
   * Implementation of solve.
   */
  template<typename Scalar, int N>
  void MultivariateGaussian<Scalar, N>::solve()
  {
    if(!solved)
    {
      Eigen::LLT<CovarianceType> llt(theCovariance);
      if(llt.info() == Eigen::Success)
      {
        transform = llt.matrixL();
      }
      else
      {
        // Covariance is not symmetric positive definite, so we'll solve via its eigenvectors.
        Eigen::SelfAdjointEigenSolver<CovarianceType> eigensolver(theCovariance);
        transform = eigensolver.eigenvectors() * eigensolver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
      }
      solved = true;
    }
  }

  /**
   * Implementation of the call operator.
   * Returns a list of samples from this distribution.
   */
  template<typename Scalar, int N>
  typename MultivariateGaussian<Scalar, N>::MeansType MultivariateGaussian<Scalar, N>::operator()(const unsigned numOfSamples)
  {
    if(!solved)
      solve();
    return (transform * MeansType::NullaryExpr(N, numOfSamples, normal)).colwise() + theMean;
  }

  /**
   * Implementation of the multivariate probability density function.
   */
  template<typename Scalar, int N>
  float MultivariateGaussian<Scalar, N>::pdf(const MeanType& point) const
  {
    const MeanType meanDiff = point - theMean;
    return 1.f / std::sqrt(std::pow(pi2, static_cast<float>(N)) * theCovariance.determinant())
           * std::exp(-0.5f * meanDiff.transpose() * theCovariance.inverse() * meanDiff);
  }
}

// SingleDimGaussian type aliases to use in the wild.
template<typename Scalar>
using Gaussian1X = impl::SingleDimGaussian<Scalar>;
using Gaussian1f = impl::SingleDimGaussian<float>;
using Gaussian1d = impl::SingleDimGaussian<double>;

// MultivariateGaussian type aliases to use in the wild.
template<typename Scalar, unsigned N>
using GaussianXX = impl::MultivariateGaussian<Scalar, N>;
template<unsigned N>
using GaussianXf = impl::MultivariateGaussian<float, N>;
template<unsigned N>
using GaussianXd = impl::MultivariateGaussian<double, N>;
using Gaussian2f = impl::MultivariateGaussian<float, 2>;
using Gaussian2d = impl::MultivariateGaussian<double, 2>;
