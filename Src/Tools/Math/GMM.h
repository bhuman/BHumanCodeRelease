/**
 * @file GMM.h
 *
 * This file defines a class that implements Gaussian Mixture Models.
 *
 * @author Jonas Kuball
 */

#pragma once

#include "Eigen.h"
#include "BHMath.h"

#include <limits>
#include "Tools/Debugging/Debugging.h"

namespace impl
{
  /**
   * @class SingleDimGMM
   * This class implements single-dimensional Gaussian Mixture Models, which can be trained by the Expectation Maximization algorithm.
   *
   * Template Parameters:
   *  Scalar: Type of internal accuracy. Usually float. Sometimes double. Never std::string.
   */
  template<typename Scalar>
  class SingleDimGMM
  {
  public:
    using MeanType = Scalar;
    using MeansType = Eigen::Array<MeanType, Eigen::Dynamic, 1>;
    using WeightType = Scalar;
    using WeightsType = Eigen::Array<WeightType, Eigen::Dynamic, 1>;
    using StdDevType = Scalar;
    using StdDevsType = Eigen::Array<StdDevType, Eigen::Dynamic, 1>;
    using VarianceType = Scalar;
    using VariancesType = Eigen::Array<VarianceType, Eigen::Dynamic, 1>;

    /**
     * No no-param-constructor.
     */
    SingleDimGMM() = delete;

    /**
     * Declaration of the default parameter constructor.
     * Initializes a given number of components with standard deviations, the same weights but slightly different means.
     */
    SingleDimGMM(const int numOfComponents);

    /**
     * The probability density function.
     */
    Scalar pdf(const MeanType& sample) const;

    /**
     * Predict function.
     *
     * @param sample The sample to predict for.
     * @param probability Optionally a Scalar adress to write the best matching probability to.
     * @return The index of the best matching component.
     */
    Eigen::Index predict(const MeanType& sample, Scalar* probability = nullptr) const;

    /**
     * Apply the EM step once.
     * @param logLikelihood If an address is given, after the EM steps the log likelihood is calculated and returned.
     */
    void em(const MeansType& samples, Scalar* logLikelihood = nullptr);

    /**
     * Repeatedly apply the EM-Algorithm until convergence is reached.
     * Convergence is checked by maximizing the log-likelihood.
     * Warning: This is massively slow and nothing you want to do every cycle.
     * If you want to train something at runtime, please re-implement this over multiple update()'s.
     */
    void train(const MeansType& samples, const Scalar epsilon = std::numeric_limits<Scalar>::epsilon());

    // Read-Only getters
    int numOfComponents() const { return theNumOfComponents; };
    const WeightsType& weights() const { return theWeights; };
    const MeansType& means() const { return theMeans; };
    const StdDevsType& stdDevs() const { return theStdDevs; };
    const VariancesType& variances() const { return theVariances; };

  protected:
    /**
     * Returns the weighted pdfs for each component.
     */
    const Eigen::Array<Scalar, Eigen::Dynamic, 1> wpdfs(const MeanType& sample) const;

    // Attributes
    int theNumOfComponents;
    WeightsType theWeights;
    MeansType theMeans;
    StdDevsType theStdDevs;
    VariancesType theVariances;
  };

  /**
   * Implementation of the parameter constructor.
   * Initializes a given number of components with standard deviations, the same weights but slightly different means.
   */
  template<typename Scalar>
  SingleDimGMM<Scalar>::SingleDimGMM(const int numOfComponents)
    : theNumOfComponents(numOfComponents), theWeights(numOfComponents), theMeans(numOfComponents), theStdDevs(numOfComponents), theVariances(numOfComponents)
  {
    theWeights.setConstant(1.f / numOfComponents);
    for(int i = 0; i < numOfComponents; ++i)
      theMeans(i) = 100.f * i;
    theStdDevs.setOnes(numOfComponents);
    theVariances.setOnes(numOfComponents);
  }

  /**
   * Implementation of the probability density function.
   * Warning: CAN return 0.
   */
  template<typename Scalar>
  Scalar SingleDimGMM<Scalar>::pdf(const MeanType& sample) const
  {
    return (this->wpdfs(sample)).sum();
  }

  /**
   * Implementation of the internal weighted-pdfs-function.
   * Warning: CAN return 0's.
   */
  template<typename Scalar>
  const Eigen::Array<Scalar, Eigen::Dynamic, 1> SingleDimGMM<Scalar>::wpdfs(const MeanType& sample) const
  {
    return (theWeights * (theStdDevs * std::sqrt(Constants::pi2)).cwiseInverse() * (-(((sample - theMeans) / theStdDevs).square() / 2)).exp());
  }

  /**
   * Implementation of the predict function.
   */
  template<typename Scalar>
  Eigen::Index SingleDimGMM<Scalar>::predict(const MeanType& sample, Scalar* probability) const
  {
    Eigen::Index bestMatch;
    const Eigen::Array<Scalar, Eigen::Dynamic, 1>& wpdfs = this->wpdfs(sample);
    wpdfs.maxCoeff(&bestMatch);
    if(probability != nullptr)
      *probability = wpdfs(bestMatch);
    return bestMatch;
  }

  /**
   * TODO: Documentation
   */
  template<typename Scalar>
  void SingleDimGMM<Scalar>::train(const MeansType& samples, const Scalar epsilon)
  {
    Scalar logLikelihood = std::numeric_limits<Scalar>::min(), lastLogLikelihood;
    do
    {
      lastLogLikelihood = logLikelihood;
      this->em(samples, &logLikelihood);
    }
    while(std::abs(lastLogLikelihood - logLikelihood) > epsilon);
  }

  /**
   * TODO: Documentation
   */
  template<typename Scalar>
  void SingleDimGMM<Scalar>::em(const MeansType& samples, Scalar* logLikelihood)
  {
    using Array = Eigen::Array<Scalar, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    const bool shouldCalculateLogLikelihood(logLikelihood != nullptr);
    Array logLikelihoods;
    if(shouldCalculateLogLikelihood)
      logLikelihoods = Array::Zero(theNumOfComponents);

    // Expectation Step: Calculate the responsibilities.
    Matrix responsibilities(samples.rows(), theNumOfComponents);
    for(int index = 0; index < samples.rows(); ++index)
    {
      const Array wpdfs(this->wpdfs(samples(index)).cwiseMax(std::numeric_limits<Scalar>::epsilon()));
      if(shouldCalculateLogLikelihood)
        logLikelihoods += wpdfs;

      responsibilities.row(index) = wpdfs / wpdfs.sum();
    }

    if(shouldCalculateLogLikelihood)
      *logLikelihood = (logLikelihoods.log().sum() / theNumOfComponents);

    // Maximization Step: Calculate new parameter sets for the components to match the expectations.
    const Array responsibilitySums(responsibilities.colwise().sum());
    const Array responsibilitySumsInverse(responsibilitySums.inverse());
    theMeans = responsibilitySumsInverse * (responsibilities.colwise() * samples).colwise().sum().transpose();
    theWeights = responsibilitySums / samples.size();
    for(int cindex = 0; cindex < theNumOfComponents; ++cindex) // TODO: Make this an optimized Eigen-Statement.
    {
      Scalar sum = 0;
      for(int sindex = 0; sindex < samples.size(); ++sindex)
      {
        sum += responsibilities(sindex, cindex) * sqr(samples(sindex) - theMeans(cindex));
      }
      theVariances(cindex) = sum;
    }
    theVariances *= responsibilitySumsInverse;
    theStdDevs = theVariances.sqrt();
  }
}

namespace impl
{
  /**
   * @class MultiDimGMM
   * This class implements multi-dimensional Gaussian Mixture Models, which can be trained by the Expectation Maximization algorithm.
   *
   * Template Parameters:
   *  Scalar: Type of internal accuracy. Usually float. Sometimes double. Never std::string.
   *  DIM: The Dimensionality of the GMM.
   */
  template<typename Scalar, int DIM>
  class MultiDimGMM
  {
  public:
    using MeanType = Eigen::Matrix<Scalar, DIM, 1>;
    using MeansType = Eigen::Array<Scalar, Eigen::Dynamic, DIM>;
    using WeightType = Scalar;
    using WeightsType = Eigen::Array<WeightType, Eigen::Dynamic, 1>;
    using CovarianceType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using CovariancesType = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using CovarianceFlatType = Eigen::Array<Scalar, Eigen::Dynamic, 1>;

    /**
     * No no-param-constructor.
     */
    MultiDimGMM() = delete;

    /**
     * Default constructor.
     */
    MultiDimGMM(const int numOfComponents);

    /**
     * The probability density function.
     */
    Scalar pdf(const MeanType& sample) const;

    /**
     * Apply the EM step once.
     * @param logLikelihood If an address is given, after the EM steps the log likelihood is calculated and returned.
     */
    void em(const MeansType& samples, Scalar* logLikelihood = nullptr);

    /**
     * Repeatedly apply the EM-Algorithm until convergence is reached.
     * Convergence is checked by maximizing the log-likelihood.
     * Warning: This is massively slow and nothing you want to do every cycle.
     * If you want to train something at runtime, please re-implement this over multiple update()'s.
     */
    void train(const MeansType& samples, const Scalar epsilon = std::numeric_limits<Scalar>::epsilon());

    /**
     * Predict function.
     *
     * @param sample The sample to predict for.
     * @param probability Optionally a Scalar adress to write the best matching probability to.
     * @return The index of the best matching component.
     */
    Eigen::Index predict(const MeanType& sample, Scalar* probability) const;

    // Read-Only getters
    int numOfComponents() const { return theNumOfComponents; };
    const WeightsType& weights() const { return theWeights; };
    const MeansType& means() const { return theMeans; };
    const CovariancesType& covariances() const { return theCovariances; };

  protected:
    /**
     * Returns the weighted pdfs for each component.
     */
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> wpdfs(const MeanType& sample) const;

    // Attributes
    int theNumOfComponents;
    WeightsType theWeights;
    MeansType theMeans;
    CovariancesType theCovariances;
  };

  /**
   * Implementation of the parameter constructor.
   */
  template<typename Scalar, int DIM>
  MultiDimGMM<Scalar, DIM>::MultiDimGMM(const int numOfComponents)
    : theNumOfComponents(numOfComponents), theWeights(numOfComponents), theMeans(numOfComponents, DIM), theCovariances(numOfComponents, DIM * DIM)
  {
    theWeights.setConstant(1.f / numOfComponents);
    MeanType vec = MeanType::Zero();
    const MeanType ones = MeanType::Ones();
    for(int i = 0; i < theNumOfComponents; ++i, vec += 50 * ones)
      theMeans.row(i) = vec;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cov = CovarianceType::Identity(DIM, DIM);
    cov.resize(1, DIM * DIM);
    for(int i = 0; i < theNumOfComponents; ++i)
      theCovariances.row(i) = cov;
  }

  /**
   * Implementation of the probabilitiy density function.
   * Warning: CAN return 0.
   */
  template<typename Scalar, int DIM>
  Scalar MultiDimGMM<Scalar, DIM>::pdf(const MeanType& sample) const
  {
    return (this->wpdfs(sample)).sum();
  }

  /**
   * Implementation of the internal weighted-pdfs-function.
   * Warning: CAN return 0's.
   */
  template<typename Scalar, int DIM>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> MultiDimGMM<Scalar, DIM>::wpdfs(const MeanType& sample) const
  {
    using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector = Eigen::Array<Scalar, Eigen::Dynamic, 1>;
    Vector pdfs(theNumOfComponents);
    for(int cindex = 0; cindex < theNumOfComponents; ++cindex)
    {
      auto meanDiff(sample - MeanType(theMeans.row(cindex)));
      const Eigen::Map<const Matrix> cov(static_cast<const Scalar*>(theCovariances.row(cindex).data()), DIM, DIM);
      float pid = std::pow(Constants::pi2, DIM);
      pdfs(cindex) = (1.f / std::sqrt(pid * cov.determinant())) * std::exp(static_cast<Scalar>(-0.5) * static_cast<Scalar>(meanDiff.transpose() * cov.inverse() * meanDiff));
    }
    return (pdfs * theWeights);
  }

  /**
   * TODO: Documentation
   */
  template<typename Scalar, int DIM>
  void MultiDimGMM<Scalar, DIM>::train(const MeansType& samples, const Scalar epsilon)
  {
    Scalar logLikelihood = std::numeric_limits<Scalar>::min(), lastLogLikelihood;
    do
    {
      lastLogLikelihood = logLikelihood;
      this->em(samples, &logLikelihood);
    }
    while(std::abs(lastLogLikelihood - logLikelihood) > epsilon);
  }

  /**
   * TODO: Documentation
   */
  template<typename Scalar, int DIM>
  void MultiDimGMM<Scalar, DIM>::em(const MeansType& samples, Scalar* logLikelihood)
  {
    using Array = Eigen::Array<Scalar, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    const bool shouldCalculateLogLikelihood(logLikelihood != nullptr);
    Array logLikelihoods;
    if(shouldCalculateLogLikelihood)
      logLikelihoods = Array::Zero(theNumOfComponents);

    // Expectation Step: Calculate the responsibilities.
    int numOfSamples = samples.rows();
    Matrix responsibilities(numOfSamples, theNumOfComponents);
    for(int sindex = 0; sindex < numOfSamples; ++sindex)
    {
      const Array wpdfs(this->wpdfs(samples.row(sindex)).cwiseMax(std::numeric_limits<Scalar>::epsilon()));
      if(shouldCalculateLogLikelihood)
        logLikelihoods += wpdfs;
      responsibilities.row(sindex) = wpdfs / wpdfs.sum();
    }

    if(shouldCalculateLogLikelihood)
      *logLikelihood = (logLikelihoods.log().sum() / theNumOfComponents);

    // Maximization Step: Calculate new parameter sets for the components to match the expectations.
    const Array responsibilitySums(responsibilities.colwise().sum());
    const Array responsibilitySumsInverse(responsibilitySums.inverse());

    theWeights = responsibilitySums / samples.size();
    // TODO: Optimize Means calculation with eigen statements.
    for(int cindex = 0; cindex < theNumOfComponents; ++cindex)
    {
      for(int dim = 0; dim < DIM; ++dim)
      {
        Scalar sum = 0;
        for(int sindex = 0; sindex < samples.rows(); ++sindex)
          sum += responsibilities(sindex, cindex) * samples(sindex, dim);
        theMeans(cindex, dim) = responsibilitySumsInverse(cindex) * sum;
      }
    }
    // TODO: Optimize Covariance calculation with eigen statements.
    for(int cindex = 0; cindex < theNumOfComponents; ++cindex)
    {
      Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> cov(theCovariances.row(cindex).data(), DIM, DIM);
      for(int diml = 0; diml < DIM; ++diml)
      {
        for(int dimr = 0; dimr < DIM; ++dimr)
        {
          Scalar sum = 0;
          for(int sindex = 0; sindex < samples.rows(); ++sindex)
            sum += responsibilities(sindex, cindex) * (samples(sindex, diml) - theMeans(cindex, diml)) * (samples(sindex, dimr) - theMeans(cindex, dimr));
          cov(diml, dimr) = responsibilitySumsInverse(cindex) * sum;
        }
      }
    }
  }

  /**
   * Implementation of the predict function.
   */
  template<typename Scalar, int DIM>
  Eigen::Index MultiDimGMM<Scalar, DIM>::predict(const MeanType& sample, Scalar* probability) const
  {
    Eigen::Index bestMatch;
    const Eigen::Array<Scalar, Eigen::Dynamic, 1>& wpdfs = this->wpdfs(sample);
    wpdfs.maxCoeff(&bestMatch);
    if(probability != nullptr)
      *probability = wpdfs(bestMatch);
    return bestMatch;
  }
}

namespace impl
{
  /**
   * @class SingleDimGMM
   * This class implements single-dimensional Gaussian Mixture Models, which can be trained by the Expectation Maximization algorithm.
   *
   * Template Parameters:
   *  Scalar: Type of internal accuracy. Usually float. Sometimes double. Never std::string.
   */
  template<typename Scalar>
  class AdaptiveSingleDimGMM : public SingleDimGMM<Scalar>
  {
  public:
    using MeanType = Scalar;
    using MeansType = Eigen::Array<MeanType, Eigen::Dynamic, 1>;
    using WeightType = Scalar;
    using WeightsType = Eigen::Array<WeightType, Eigen::Dynamic, 1>;
    using StdDevType = Scalar;
    using StdDevsType = Eigen::Array<StdDevType, Eigen::Dynamic, 1>;
    using VarianceType = Scalar;
    using VariancesType = Eigen::Array<VarianceType, Eigen::Dynamic, 1>;

    /**
     * No no-param-constructor.
     */
    AdaptiveSingleDimGMM() = delete;

    /**
     * Declaration of the default parameter constructor.
     * Initializes a given number of components with standard deviations, the same weights but slightly different means.
     */
    AdaptiveSingleDimGMM(const int numOfComponents);

    /**
     * TODO
     */
    void singleEm(const MeanType& sample, const unsigned int sampleNum);

  private:

    // Attibutes for single sample training
    MeansType responsibilitySums;
    MeansType resSampleProdSums;
    MeansType meanSum;
    VariancesType varianceSums;
    unsigned int step;
  };

  /**
   * Implementation of the parameter constructor.
   * Initializes a given number of components with standard deviations, the same weights but slightly different means.
   */
  template<typename Scalar>
  AdaptiveSingleDimGMM<Scalar>::AdaptiveSingleDimGMM(const int numOfComponents)
    : SingleDimGMM<Scalar>(numOfComponents)
  {
    responsibilitySums = MeansType::Zero(numOfComponents);
    resSampleProdSums = MeansType::Zero(numOfComponents);
    meanSum = MeansType::Zero(numOfComponents);
    varianceSums = MeansType::Zero(numOfComponents);
    step = 0;
  }

  template<typename Scalar>
  void AdaptiveSingleDimGMM<Scalar>::singleEm(const MeanType& sample, const unsigned int sampleNum)
  {
    using Array = Eigen::Array<Scalar, Eigen::Dynamic, 1>;

    // Expectation Step: Calculate the responsibilitie
    const Array wpdfs(this->wpdfs(sample).cwiseMax(std::numeric_limits<Scalar>::epsilon()));
    Array responsibility = wpdfs / wpdfs.sum();

    // Maximization Step: Calculate new parameter sets for the components to match the expectations.
    responsibilitySums += responsibility;
    resSampleProdSums += responsibility * sample;
    Array var = responsibility * ((sample - this->theMeans) * (sample - this->theMeans));
    varianceSums += var;
    meanSum += responsibility * (sample - this->theMeans);

    step++;

    if(step == sampleNum)
    {
      Array oldMeans = this->theMeans;
      Array theMeans = responsibilitySums.inverse() * resSampleProdSums;
      Array theWeights = responsibilitySums / sampleNum;
      Array theVariances = varianceSums - 2 * (theMeans - oldMeans) * meanSum;
      theVariances *= responsibilitySums.inverse();
      theVariances += (theMeans - oldMeans).pow(2);
      theVariances = theVariances.cwiseMax(1);
      //theVariances = (responsibilitySums.inverse() * varianceSums) + (oldMeans*oldMeans) - (theMeans*theMeans);
      this->theMeans = (3 * this->theMeans + theMeans) / 4;
      this->theWeights = (3 * this->theWeights + theWeights) / 4;
      this->theVariances = (3 * this->theVariances + theVariances) / 4;
      this->theStdDevs = this->theVariances.sqrt();

      responsibilitySums = Array::Zero(this->theNumOfComponents);
      resSampleProdSums = Array::Zero(this->theNumOfComponents);
      meanSum = Array::Zero(this->theNumOfComponents);
      varianceSums = Array::Zero(this->theNumOfComponents);

      step = 0;
    }
  }
}

namespace impl
{
  /**
   * @class AdaptiveMultiDimGMM
   * This class implements multi-dimensional Gaussian Mixture Models, which can be trained by the Expectation Maximization algorithm.
   *
   * Template Parameters:
   *  Scalar: Type of internal accuracy. Usually float. Sometimes double. Never std::string.
   */
  template<typename Scalar, int DIM>
  class AdaptiveMultiDimGMM : public MultiDimGMM<Scalar, DIM>
  {
  public:
    using MeanType = Eigen::Matrix<Scalar, DIM, 1>;
    using MeansType = Eigen::Array<Scalar, Eigen::Dynamic, DIM>;
    using CovarianceType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using CovariancesType = Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using WeightType = Scalar;
    using WeightsType = Eigen::Array<WeightType, Eigen::Dynamic, 1>;
    using VarianceType = Eigen::Array<Scalar, DIM, 1>;
    using VariancesType = Eigen::Array<Scalar, Eigen::Dynamic, DIM>;
    // To avoid this->AttríbuteOfSuperClass
    using MultiDimGMM<Scalar, DIM>::theWeights;
    using MultiDimGMM<Scalar, DIM>::theMeans;
    using MultiDimGMM<Scalar, DIM>::theCovariances;
    using MultiDimGMM<Scalar, DIM>::theNumOfComponents;
    /**
    * No no-param-constructor.
    */
    AdaptiveMultiDimGMM() = delete;

    /**
     * Declaration of the default parameter constructor.
     * Initializes a given number of components with standard deviations, the same weights but slightly different means.
     */
    AdaptiveMultiDimGMM(const int numOfComponents);

    /**
     * adaptation algorithm for multi dim GMM by Stauffer and Grimson
     * http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=784637
     */
    void adapt(const MeanType& sample);
    void adapt(const MeanType& sample, const int fit);
    void update(const int fit);


    Scalar evaluate(const MeanType& sample, const int comp);

    void fitMeanInBounds(const int comp);
    void fitVarInBounds(const int comp);

    int predict(const MeanType& sample, const int defaultVal = -1);
    void initForColorCalibration();


    const VariancesType& variances() const { return theVariances; };
    const MeansType& stdDevs() const { return theStdDevs; };

  private:
    int activeComponents;
    VariancesType theVariances;
    MeansType theStdDevs;
    Scalar weightLearningRate;

    MeansType upperMeansBound;
    MeansType lowerMeansBound;
    VariancesType upperVarBound;
    VariancesType lowerVarBound;
  };

  template<typename Scalar, int DIM>
  AdaptiveMultiDimGMM<Scalar, DIM>::AdaptiveMultiDimGMM(const int numOfComponents)
    : MultiDimGMM<Scalar, DIM>(numOfComponents), theVariances(numOfComponents, DIM), theStdDevs(numOfComponents, DIM), upperMeansBound(numOfComponents, DIM), lowerMeansBound(numOfComponents, DIM), upperVarBound(numOfComponents, DIM), lowerVarBound(numOfComponents, DIM)
  {
    activeComponents = 3;
    weightLearningRate = 0.00001f;
    //VarianceType var = VarianceType::Constant(DIM, 1, 1);
    for(int i = 0; i < numOfComponents; ++i)
    {
      theVariances.row(i) = 0;
      theStdDevs.row(i) = 0;
    }
  }

  template<typename Scalar, int DIM>
  int AdaptiveMultiDimGMM<Scalar, DIM>::predict(const MeanType& sample, const int defaultVal)
  {
    //match sample to Gaussians
    for(int i = 0; i < activeComponents; ++i)
    {
      // check if sample matches one distribution
      MeanType stdDev = theStdDevs.row(i);
      MeanType mean = theMeans.row(i);
      
      MeanType mahanDev = (3.f * stdDev).array().sqrt() * stdDev.array();

      // check if sample matches one distribution
      bool upperFit = ((mahanDev + mean - sample).minCoeff() > 0);
      bool lowerFit = ((mahanDev - mean + sample).minCoeff() > 0);

      if(upperFit && lowerFit)
      {
        typename MeanType::Index fitIndex;
        MeansType cWiseMahanDists = (theMeans.array().rowwise() - sample.array().transpose()).square() / theStdDevs.array().square();
        (cWiseMahanDists.array().rowwise()).sum().sqrt().minCoeff(&fitIndex);
        return (int)fitIndex;
      }
    }
    return defaultVal;
  }

  template<typename Scalar, int DIM>
  void AdaptiveMultiDimGMM<Scalar, DIM>::initForColorCalibration()
  {
    // declare order white, field, black
    theMeans.row(0) << 255, 128, 0;
    theMeans.row(2) << 100, 178, 150;
    theMeans.row(1) << 0, 128, 0;

    theVariances.row(0) << 113, 384, 113;
    theVariances.row(2) << 194, 45, 194;
    theVariances.row(1) << 194, 384, 194;

    theStdDevs.row(0) << 10.6f, 19.6f, 10.6f;
    theStdDevs.row(2) << 14, 6.7f, 14;
    theStdDevs.row(1) << 14, 19.6f, 14;

    // bound initialization
    upperMeansBound.row(0) << 300, 178, 50;
    upperMeansBound.row(2) << 150, 200, 190;
    upperMeansBound.row(1) << 30, 178, 30;

    lowerMeansBound.row(0) << 205, 78, 0;
    lowerMeansBound.row(2) << 50, 140, 100;
    lowerMeansBound.row(1) << 0, 78, 0;

    upperVarBound.row(0) << 180, 480, 148;
    upperVarBound.row(2) << 194, 71, 194;
    upperVarBound.row(1) << 148, 384, 148;

    lowerVarBound.row(0) << 113, 330, 94;
    lowerVarBound.row(2) << 148, 28, 113;
    lowerVarBound.row(1) << 113, 330, 45;
  }

  template<typename Scalar, int DIM>
  void AdaptiveMultiDimGMM<Scalar, DIM>::fitMeanInBounds(const int comp)
  {
    MeanType upperFit = (theMeans.row(comp).min(upperMeansBound.row(comp)));
    MeanType lowerBound = lowerMeansBound.row(comp);
    theMeans.row(comp) = upperFit.array().max(lowerBound.array());
  }

  template<typename Scalar, int DIM>
  void AdaptiveMultiDimGMM<Scalar, DIM>::fitVarInBounds(const int comp)
  {
    VarianceType upperFit = (theVariances.row(comp).cwiseMin(upperVarBound(comp)));
    VarianceType lowerBound = lowerVarBound.row(comp);
    theVariances.row(comp) = upperFit.max(lowerBound);
  }

  template<typename Scalar, int DIM>
  Scalar AdaptiveMultiDimGMM<Scalar, DIM>::evaluate(const MeanType& sample, const int comp)
  {
    using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    auto meanDiff(sample - MeanType(theMeans.row(comp)));
    const Eigen::Map<const Matrix> cov(static_cast<const Scalar*>(theCovariances.row(comp).data()), DIM, DIM);
    double pid = std::pow(Constants::pi2, DIM);
    return (1.f / std::sqrt(static_cast<float>(pid) * cov.determinant())) * std::exp(static_cast<Scalar>(-0.5) * static_cast<Scalar>(meanDiff.transpose() * cov.inverse() * meanDiff));
  }

  template<typename Scalar, int DIM>
  void AdaptiveMultiDimGMM<Scalar, DIM>::adapt(const MeanType& sample)
  { 
    // Initial values    
    const float initialVariance = 2500.f;
    int fit = -1;

    //const float learnParameterA = 150.f;
    //const float learnParameterB = 5.f;
    fit = predict(sample); // Match sample to Gaussians

    if(fit < 0) //If no match was found initialize new component
    {
      /*Adjust weights that 1-3 are not min(>1) to make sure there is always an
        understanding of white, green and black*/
      WeightsType adjustedWeights = theWeights;
      for(int i = 0; i < 3; ++i)
        adjustedWeights(i, 0)++;

      //Get min weight index
      typename WeightsType::Index minWeightRow;
      WeightType minWeight = adjustedWeights.minCoeff(&minWeightRow);

      //Determine if component is underrepresentated
      if(minWeight > 0.04 && activeComponents < theNumOfComponents) //No underreprenstated component -> add new component
      {
        ++activeComponents;
        fit = activeComponents - 1;
      }
      else //Underrepresentated component or maximum number of components -> change for new comp with high variance
        fit = (int)minWeightRow;

      theMeans.row(fit) = sample;
      theVariances.row(fit) = VarianceType::Constant(DIM, 1, initialVariance);
      theWeights.row(fit) = 0;
      
      //Update weights, variances and covariances
      update(fit);
    }
    else
    {
      adapt(sample, fit);
    }
  }
  
  template<typename Scalar, int DIM>
  void AdaptiveMultiDimGMM<Scalar, DIM>::adapt(const MeanType& sample, const int fit)
  {
    //If match was found update matching component
    int fitIndex = static_cast<int>(fit);
    //Scalar secondLearningRate = learnParameterA * (learnParameterB * evaluate(sample, fitIndex) + std::numeric_limits<float>::min());
    Scalar secondLearningRate = weightLearningRate / theWeights(fit);
    
    Scalar invSecondLearningRate = (1.f - secondLearningRate);

    MeanType mean = theMeans.row(fit);
    mean = invSecondLearningRate * mean + secondLearningRate * sample;
    theMeans.row(fit) = mean;
    fitMeanInBounds(fitIndex);

    VarianceType variances = theVariances.row(fit);
    VarianceType varianceShift = secondLearningRate * (sample - mean).array().pow(2);
    theVariances.row(fit) = (invSecondLearningRate * variances +  varianceShift);
    fitVarInBounds(fitIndex);
    
    //Update weights, variances and covariances
    update(fit);
  }
  
  template<typename Scalar, int DIM>
  void AdaptiveMultiDimGMM<Scalar, DIM>::update(const int fit)
  {
    //Update weights, variances and covariances
    VarianceType variances = theVariances.row(fit).cwiseMax(100);
    auto theCov = theCovariances.row(fit);
    theCov(0) = variances(0);
    theCov(4) = variances(1);
    theCov(8) = variances(2);
    theCovariances.row(fit) = theCov;
    theVariances.row(fit) = variances;

    theStdDevs.row(fit) = variances.sqrt();

    theWeights = theWeights * (1.f - weightLearningRate);
    theWeights(fit) = theWeights(fit) + weightLearningRate;
    Scalar normFactor = 1.f / theWeights.sum();
    theWeights = theWeights * normFactor;
  }
}
// Type aliases for usage

// Single Dim
template<typename Scalar>
using GMM1x = impl::SingleDimGMM<Scalar>;
using GMM1f = impl::SingleDimGMM<float>;
using GMM1d = impl::SingleDimGMM<double>;

// Multi Dim
template<typename Scalar, int DIM>
using GMMxx = impl::MultiDimGMM<Scalar, DIM>;
using GMM2f = impl::MultiDimGMM<float, 2>;
using GMM2d = impl::MultiDimGMM<double, 2>;
using GMM3f = impl::MultiDimGMM<float, 3>;
using GMM3d = impl::MultiDimGMM<double, 3>;
