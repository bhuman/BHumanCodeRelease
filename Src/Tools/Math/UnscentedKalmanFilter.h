/**
 * @file Tools/Math/UnscentedKalmanFilter.h
 *
 * A generic implementation of an Unscented Kalman Filter.
 *
 * @author <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>
 */

#pragma once

#include "Approx.h"
#include "BHMath.h"
#include "Eigen.h"
#include "Platform/BHAssert.h"

#include <functional>
#include <limits>

/**
 * A Manifold construct to handle state spaces with singularities and encapsualte
 * the topological stucure with [+] and [-] operations.
 */
template<unsigned N>
struct Manifold
{
  static constexpr unsigned DOF = N;
  using Vectorf = Eigen::Matrix<float, DOF, 1>; // Handle the element like a vector

  Manifold operator+(const Vectorf& vec) const; // The [+] operator
  Manifold& operator+=(const Vectorf& vec);
  Vectorf operator-(const Manifold& other) const; // The [-] operator
};

namespace impl
{
  /**
   * A struct to run a calculation of mean for given sigma points.
   */
  template<typename State, unsigned DOF, typename Array, bool IsManifold>
  struct MeanOfSigmaPoints
  {
    static State run(const Array& sigmaPoints);
  };

  /**
   * The class for the Unscented Kalman Filter for hypotheses generation by using
   * Kalman filtering using Sigma Points.
   */
  template<typename State, unsigned DOF, bool Manifold>
  class UnscentedKalmanFilter
  {
  public:
    using CovarianceType = Eigen::Matrix<float, DOF, DOF>; // The covariance size to use
    template<unsigned N>
    using Vectorf = Eigen::Matrix<float, N, 1>; // The vector size to use

  private:
    template<typename T>
    using SigmaArray = std::array < T, DOF * 2 + 1 >; // The array type for the sigma points

  public:
    State mean; // The mean of the hypothesis that is generated
    CovarianceType cov = CovarianceType::Zero(); // The covariance of the hypothesis to quantify the certainty

  private:
    SigmaArray<State> sigmaPoints; // The array for the sigma points

  public:
    /**
     * The constructor for the filter that requires an initial state to start with.
     * @param initState, the state to initialize the filter
     */
    UnscentedKalmanFilter(const State& initState);

    /**
     * Initalization function to start the process. Setting the mean, covariance and sigma points.
     * @param initState, the mean to be set
     * @param initNoise, the noise (as a variance) to initialize the covariance
     */
    void init(const State& initState, const CovarianceType& initNoise);

    /**
     * The prediction step to propagate the whole hypothesis with a given dynamic model and an operation specific noise.
     * In other works this function is referred as dynamic step.
     * @param dynamicModel, a function to propagate the state
     * @param noise, the propagation specific noise (as a variance) to quantify the uncertainty
     */
    void predict(std::function<void(State&)> dynamicModel, const CovarianceType& noise);

    /**
     * The multi dimensional update step to integrate a measurement into an existing hypothesis.
     * In other works this function is referred as measurement step.
     * @param measurement, a vector that stores all relevant data of a measurement
     * @param measurementModel, a function that returns a measurement for a state
     * @param measurementNoise, the measurement specific noise (as a variance) to quantify the uncertainty
     */
    template<unsigned N>
    void update(const Vectorf<N>& measurement, std::function<Vectorf<N>(const State&)> measurementModel, const Eigen::Matrix<float, N, N>& measurementNoise);

    /**
     * The single dimensional update step to integrate a measurement into an existing hypothesis.
     * In other works this function is referred as measurement step.
     * @param measurement, a float value that represents a measurement
     * @param measurementModel, a function that returns a measurement for a state
     * @param measurementNoise, the measurement specific noise (as a variance) to quantify the uncertainty
     */
    void update(float measurement, std::function<float(const State&)> measurementModel, float measurementNoise);

  private:
    /**
     * The helper function to calculate sigma points using the given mean and the covariance.
     */
    void updateSigmaPoints();

    /**
     * The function to call a new mean out of the sigma points.
     * @return the state that is the new mean
     */
    State meanOfSigmaPoints() const;

    /**
     * A helper function to validate if a covariance is still a covariance.
     * The validation is done via asserts.
     * @param cov, the covariance to check
     */
    void covarianceMatrixValidation(const CovarianceType& cov) const;

    /**
     * A helper function to fix a covariance by forcing the symmetric property.
     * @param cov, the covariance to fix
     */
    void fixCovarianceMatrix(CovarianceType& cov);
  };

  /**
   * The constructor for the filter that requires an initial state to start with.
   * @param initState, the state to initialize the filter
   */
  template<typename State, unsigned DOF, bool Manifold>
  UnscentedKalmanFilter<State, DOF, Manifold>::UnscentedKalmanFilter(const State& initState) :
    mean(initState)
  {
    sigmaPoints.fill(initState);
  }

  /**
   * Initalization function to start the process. Setting the mean, covariance and sigma points.
   * @param initState, the mean to be set
   * @param initNoise, the noise (as a variance) to initialize the covariance
   */
  template<typename State, unsigned DOF, bool Manifold>
  void UnscentedKalmanFilter<State, DOF, Manifold>::init(const State& initState, const CovarianceType& initNoise)
  {
    mean = initState;
    cov = initNoise;
    sigmaPoints.fill(initState);
  }

  /**
   * The prediction step to propagate the whole hypothesis with a given dynamic model and an operation specific noise.
   * In other works this function is referred as dynamic step.
   * @param dynamicModel, a function to propagate the state
   * @param noise, the propagation specific noise (as variance) to quantify the uncertainty
   */
  template<typename State, unsigned DOF, bool Manifold>
  void UnscentedKalmanFilter<State, DOF, Manifold>::predict(std::function<void(State&)> dynamicModel, const CovarianceType& noise)
  {
    ASSERT((noise.array() >= 0.f).all());
    ASSERT(noise.trace() > 0.f);

    updateSigmaPoints();

    for(State& sigmaPoint : sigmaPoints)
      dynamicModel(sigmaPoint);

    mean = meanOfSigmaPoints();

    cov = CovarianceType::Zero();
    for(State& sigmaPoint : sigmaPoints)
    {
      const Vectorf<DOF> dist = sigmaPoint - mean;
      cov += dist * dist.transpose();
    }
    cov *= 0.5f;
    cov += noise;

    fixCovarianceMatrix(cov);
    covarianceMatrixValidation(cov);
  }

  /**
   * The multi dimensional update step to integrate a measurement into an existing hypothesis.
   * In other works this function is referred as measurement step.
   * @param measurement, a vector that stores all relevant data of a measurement
   * @param measurementModel, a function that returns a measurement for a state
   * @param measurementNoise, the measurement specific noise (as variance) to quantify the uncertainty
   */
  template<typename State, unsigned DOF, bool IsManifold>
  template<unsigned N>
  void UnscentedKalmanFilter<State, DOF, IsManifold>::update(const Vectorf<N>& measurement, std::function<Vectorf<N>(const State&)> measurementModel, const Eigen::Matrix<float, N, N>& measurementNoise)
  {
    ASSERT((measurementNoise.diagonal().array() >= 0.f).all());
    ASSERT(measurementNoise.trace() > 0.f);

    using MeasurementType = Vectorf<N>;
    using MeasurementCovarianceType = Eigen::Matrix<float, N, N>;
    using MixedCovarianceType = Eigen::Matrix<float, DOF, N>;

    updateSigmaPoints();

    SigmaArray<MeasurementType> Z;
    for(size_t i = 0; i < sigmaPoints.size(); ++i)
      Z[i] = measurementModel(sigmaPoints[i]);

    MeasurementType z = MeasurementType::Zero();
    for(MeasurementType& Zi : Z)
      z += Zi;
    z /= static_cast<float>(sigmaPoints.size());

    MeasurementCovarianceType sigmaz = MeasurementCovarianceType::Zero();
    for(MeasurementType& Zi : Z)
    {
      const MeasurementType dist = Zi - z;
      sigmaz += dist * dist.transpose();
    }
    sigmaz *= 0.5f;
    sigmaz += measurementNoise;

    MixedCovarianceType simgaxz = MixedCovarianceType::Zero();
    for(size_t i = 0; i < sigmaPoints.size(); ++i)
    {
      const State& Xi = sigmaPoints[i];
      const MeasurementType& Zi = Z[i];
      simgaxz += (Xi - mean) * (Zi - z).transpose();
    }
    simgaxz *= 0.5f;
    //The kalman gain
    const MixedCovarianceType K = simgaxz * sigmaz.inverse();
    //Derive the mean and the covariance using the kalman gain
    mean += K * (measurement - z);
    cov -= K * sigmaz * K.transpose();

    fixCovarianceMatrix(cov);
    covarianceMatrixValidation(cov);
  }

  /**
   * The single dimensional update step to integrate a measurement into an existing hypothesis.
   * In other works this function is referred as measurement step.
   * @param measurement, a float value that represents a measurement
   * @param measurementModel, a function that returns a measurement for a state
   * @param measurementNoise, the measurement specific noise (as variance) to quantify the uncertainty
   */
  template<typename State, unsigned DOF, bool IsManifold>
  void UnscentedKalmanFilter<State, DOF, IsManifold>::update(float measurement, std::function<float(const State&)> measurementModel, float measurementNoise)
  {
    ASSERT(measurementNoise > 0.f);

    using MixedCovarianceType = Eigen::Matrix<float, DOF, 1>;

    updateSigmaPoints();

    SigmaArray<float> Z;
    for(size_t i = 0; i < sigmaPoints.size(); ++i)
      Z[i] = measurementModel(sigmaPoints[i]);

    float z = 0.f;
    for(float& Zi : Z)
      z += Zi;
    z /= static_cast<float>(sigmaPoints.size());

    float sigmaz = 0.f;
    for(float& Zi : Z)
      sigmaz += sqr(Zi - z);
    sigmaz *= 0.5f;
    sigmaz += measurementNoise;

    MixedCovarianceType simgaxz = MixedCovarianceType::Zero();
    for(size_t i = 0; i < sigmaPoints.size(); ++i)
    {
      const State& Xi = sigmaPoints[i];
      const float& Zi = Z[i];
      simgaxz += (Xi - mean) * (Zi - z);
    }
    simgaxz *= 0.5f;

    const MixedCovarianceType K = simgaxz * (1.f / sigmaz);

    mean += K * (measurement - z);
    cov -= K * sigmaz * K.transpose();

    fixCovarianceMatrix(cov);
    covarianceMatrixValidation(cov);
  }

  /**
   * The helper function to calculate sigma points using the given mean and the covariance.
   */
  template<typename State, unsigned DOF, bool IsManifold>
  void UnscentedKalmanFilter<State, DOF, IsManifold>::updateSigmaPoints()
  {
    Eigen::LLT<CovarianceType> llt = cov.llt();
    Eigen::ComputationInfo info = llt.info();
    if(info == Eigen::ComputationInfo::Success)
    {
      CovarianceType l = llt.matrixL();
      sigmaPoints[0] = mean;
      for(unsigned i = 0; i < DOF; ++i)
        sigmaPoints[i + 1] = mean + l.col(i);
      for(unsigned i = 0; i < DOF; ++i)
        sigmaPoints[i + DOF + 1] = mean + (-l.col(i));
    }
    else
    {
      // maybe adding 1 sigma or something is better?
      for(State& sigmaPoint : sigmaPoints)
        sigmaPoint = mean;
    }
  }

  /**
   * The function to call a new mean out of the sigma points.
   * @return the state that is the new mean
   */
  template<typename State, unsigned DOF, bool IsManifold>
  inline State UnscentedKalmanFilter<State, DOF, IsManifold>::meanOfSigmaPoints() const
  {
    return MeanOfSigmaPoints<State, DOF, SigmaArray<State>, IsManifold>::run(sigmaPoints);
  }

  /**
   * A helper function to validate if a covariance is still a covariance.
   * The validation is done via asserts.
   * @param cov, the covariance to check
   */
  template<typename State, unsigned DOF, bool IsManifold>
  inline void UnscentedKalmanFilter<State, DOF, IsManifold>::covarianceMatrixValidation(const CovarianceType& cov) const
  {
    for(unsigned i = 0; i < DOF; ++i)
    {
      ASSERT(cov(i, i) > 0.f);
      ASSERT(std::isfinite(cov(i, i)));
      ASSERT(std::isnormal(cov(i, i)));

      for(unsigned j = i + 1; j < DOF; ++j)
      {
        ASSERT(cov(i, j) == cov(j, i));
        ASSERT(std::isfinite(cov(j, i)));
      }
    }
  }

  /**
   * A helper function to fix a covariance by forcing the symmetric property.
   * @param cov, the covariance to fix
   */
  template<typename State, unsigned DOF, bool IsManifold>
  inline void UnscentedKalmanFilter<State, DOF, IsManifold>::fixCovarianceMatrix(CovarianceType& cov)
  {
    for(unsigned i = 0; i < DOF; ++i)
    {
      for(unsigned j = i + 1; j < DOF; ++j)
      {
        cov(i, j) = cov(j, i) = (cov(i, j) + cov(j, i)) * .5f;
      }
    }
  }

  /**
   * A struct to run a calculation of mean for given sigma points with manifold.
   */
  template<typename State, unsigned DOF, typename Array>
  struct MeanOfSigmaPoints<State, DOF, Array, true>
  {
    using Vectorf = Eigen::Matrix<float, DOF, 1>; // The vector type to operate on

    /**
     * Calculate the mean of states via manifold convergence algorithm.
     * @param sigmaPoints, an array of states to get a mean of
     * @return the mean
     */
    static State run(const Array& sigmaPoints)
    {
      State mean = sigmaPoints[0]; // The mean to use for convergence
      State lastMean; // The mean to check convergence with
      unsigned iterations = 0; // The iteration counter
      // Move the mean each step into the direction of the sigma points till the difference is minimal or the limit is reached
      do
      {
        lastMean = mean;
        ++iterations;
        Vectorf sum = Vectorf::Zero();
        for(const State& sigmaPoint : sigmaPoints)
          sum += sigmaPoint - mean;
        mean += sum / static_cast<float>(sigmaPoints.size());
      }
      while(!Approx::isZero((lastMean - mean).norm()) && iterations <= 20);
      return mean;
    }
  };

  /**
   * A struct to run a calculation of mean for given sigma points without manifold.
   */
  template<typename State, unsigned DOF, typename Array>
  struct MeanOfSigmaPoints<State, DOF, Array, false>
  {
    /**
     * Calculate the arithmetic mean of vectors.
     * @param sigmaPoints, an array of vectors to get a mean of
     * @return the mean
     */
    static State run(const Array& sigmaPoints)
    {
      State sum = State::Zero();
      for(const State& sigmaPoint : sigmaPoints)
        sum += sigmaPoint;
      return sum / static_cast<float>(sigmaPoints.size());
    }
  };
}

/**
 * UKF type for easy acces with and without manifold.
 */
template<unsigned DOF>
using UKF = impl::UnscentedKalmanFilter<Eigen::Matrix<float, DOF, 1>, DOF, false>;

template<typename Manifold_>
using UKFM = impl::UnscentedKalmanFilter<Manifold_, Manifold_::DOF, true>;
