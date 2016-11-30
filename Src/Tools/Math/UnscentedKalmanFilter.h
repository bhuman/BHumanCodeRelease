/**
 * @file Tools/Math/UnscentedKalmanFilter.h
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

template<unsigned N>
struct Manifold
{
  static constexpr unsigned DOF = N;
  using Vectorf = Eigen::Matrix<float, DOF, 1>;

  Manifold operator+(const Vectorf& vec) const;
  Manifold& operator+=(const Vectorf& vec);
  Vectorf operator-(const Manifold& other) const;
};

namespace impl
{
  template<typename State, unsigned DOF, typename Array, bool IsManifold>
  struct MeanOfSigmaPoints
  {
    static State run(const Array& sigmaPoints);
  };

  template<typename State, unsigned DOF, bool Manifold>
  class UnscentedKalmanFilter
  {
  public:
    using CovarianceType = Eigen::Matrix<float, DOF, DOF>;
    template<unsigned N>
    using Vectorf = Eigen::Matrix<float, N, 1>;

  private:
    template<typename T>
    using SigmaArray = std::array<T, DOF * 2 + 1>;

  public:
    State mean;
    CovarianceType cov = CovarianceType::Zero();

  private:
    SigmaArray<State> sigmaPoints;

  public:
    UnscentedKalmanFilter(const State& initState);

    void init(const State& initState, const CovarianceType& initNoise);

    void predict(std::function<void(State&)> dynamicModel, const CovarianceType& noise);

    template<unsigned N>
    void update(const Vectorf<N>& measurement, std::function<Vectorf<N>(const State&)> measurementModel, const Eigen::Matrix<float, N, N>& measurementNoise);
    void update(float measurement, std::function<float(const State&)> measurementModel, float measurementNoise);

  private:
    void updateSigmaPoints();
    State meanOfSigmaPoints() const;
    void covarianceMatrixValidation(const CovarianceType& cov) const;
    void fixCovarianceMatrix(CovarianceType& cov);
  };

  template<typename State, unsigned DOF, bool Manifold>
  UnscentedKalmanFilter<State, DOF, Manifold>::UnscentedKalmanFilter(const State& initState) :
    mean(initState)
  {
    sigmaPoints.fill(initState);
  }

  template<typename State, unsigned DOF, bool Manifold>
  void UnscentedKalmanFilter<State, DOF, Manifold>::init(const State& initState, const CovarianceType& initNoise)
  {
    mean = initState;
    cov = initNoise.array().square().matrix();
    sigmaPoints.fill(initState);
  }

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
    cov += noise.cwiseAbs2();

    fixCovarianceMatrix(cov);
    covarianceMatrixValidation(cov);
  }

  template<typename State, unsigned DOF, bool IsManifold>
  template<unsigned N>
  void UnscentedKalmanFilter<State, DOF, IsManifold>::update(const Vectorf<N>& measurement, std::function<Vectorf<N>(const State&)> measurementModel, const Eigen::Matrix<float, N, N>& measurementNoise)
  {
    ASSERT((measurementNoise.array() >= 0.f).all());
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
    sigmaz += measurementNoise.cwiseAbs2();

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

  template<typename State, unsigned DOF, bool IsManifold>
  void UnscentedKalmanFilter<State, DOF, IsManifold>::update(float measurement, std::function<float(const State&)> measurementModel, float measurementNoise)
  {
    ASSERT(measurementNoise > 0.f);

    using MixedCovarianceType = Eigen::Matrix<float, State::DOF, 1>;

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
    sigmaz += sqr(measurementNoise);

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

  template<typename State, unsigned DOF, bool IsManifold>
  inline State UnscentedKalmanFilter<State, DOF, IsManifold>::meanOfSigmaPoints() const
  {
    return MeanOfSigmaPoints<State, DOF, SigmaArray<State>, IsManifold>::run(sigmaPoints);
  }

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

  template<typename State, unsigned DOF, typename Array>
  struct MeanOfSigmaPoints<State, DOF, Array, true>
  {
    using Vectorf = Eigen::Matrix<float, DOF, 1>;

    static State run(const Array& sigmaPoints)
    {
      State mean = sigmaPoints[0];
      State lastMean;
      unsigned iterations = 0;
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

  template<typename State, unsigned DOF, typename Array>
  struct MeanOfSigmaPoints<State, DOF, Array, false>
  {
    static State run(const Array& sigmaPoints)
    {
      State sum = State::Zero();
      for(const State& sigmaPoint : sigmaPoints)
        sum += sigmaPoint;
      return sum / static_cast<float>(sigmaPoints.size());
    }
  };
}

template<unsigned DOF>
using UKF = impl::UnscentedKalmanFilter<Eigen::Matrix<float, DOF, 1>, DOF, false>;

template<typename Manifold_>
using UKFM = impl::UnscentedKalmanFilter<Manifold_, Manifold_::DOF, true>;
