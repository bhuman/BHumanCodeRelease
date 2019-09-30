/**
 * @file BallStateEstimateFilters.h
 *
 * Different state estimators for rolling and stationary balls.
 * Currently based on a standard Kalman filters
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Debugging/Debugging.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Modeling/BallLocatorTools.h"

/**
 * Generic base class for estimates.
 * Provides just a few members and functions to
 * avoid the duplication of code
 */
class BallStateEstimate
{
public:
  float radius = 1.f;                        /**< Last perceived radius of the ball */
  int numOfMeasurements = 0;                 /**< The number of measurements that have been fused in this hypothesis */
  float likelihoodOfMeasurements = 0.f;      /**< Product of likelihoods of integrated measurements, scaled in relation to hypothesis with highest value (which then has a value of 1) */
  float weighting = 0.f;                     /**< Kind of likelihood of hypothesis (likelihood of measurements multiplied with likelihood at mean of covariance) */
  Vector2f lastPosition = Vector2f::Zero();  /**< Store position before update, needed for collision detection */

  /**
   * Make sure that destructors of derived objects are virtual.
   */
  virtual ~BallStateEstimate() = default;

  /**
   * Updates hypothesis based on a new measurement
   * and thus updates mean and covariance as well as weight, height, and radius
   * @param measurement The ball perception in relative field coordinates
   * @param measurementCov The covariance of the measurement
   * @param radius The radius of the perceived ball (for scenarios with variable ball sizes)
   */
  void measurementUpdate(const Vector2f& measurement, const Matrix2f& measurementCov, float radius)
  {
    // Compute quality of estimate (w.r.t. the current measurement) before integrating the measurement:
    ASSERT(likelihoodOfMeasurements >= 0.f);
    const Vector2f x = getPosition();
    const Matrix2f P = getPositionCovariance();
    float gain = BallLocatorTools::getLikelihoodOfPosition(measurement, measurementCov, x); // Probability of mean w.r.t. current measurement
    if(!(gain >= 0.f && gain <= 10.f))
    {
      OUTPUT_ERROR("getLikelihoodOfPosition: gain=" << gain);
      OUTPUT_ERROR("measurement x=" << measurement.x() << " y=" << measurement.y());
      OUTPUT_ERROR("measurementCov r0c0=" << measurementCov(0, 0) << " r0c1=" << measurementCov(0, 1) << " r1c0=" << measurementCov(1, 0) << " r1c1=" << measurementCov(1, 1));
      OUTPUT_ERROR("x x.x=" << x[0] << " x.y=" << x[1]);
    }
    ASSERT(gain >= 0.f && gain <= 10.f);
    likelihoodOfMeasurements *= gain;
    weighting = likelihoodOfMeasurements * BallLocatorTools::getLikelihoodOfMean(P);

    // Major estimation step, performed by subclass:
    integrateMeasurement(measurement, measurementCov);
    // Update statistics and ball radius:
    this->radius = radius;
    ++numOfMeasurements;
  }

  /**
   * Measurement integration step of the filter.
   * Needs to be implemented by each subclass.
   * @param measurement The ball perception in relative field coordinates
   * @param measurementCov The covariance of the measurement
   */
  virtual void integrateMeasurement(const Vector2f& measurement, const Matrix2f& measurementCov) = 0;

  /**
   * Every instance of a filter has to implement this function for getting the ball estimate position
   * @return The position of the ball (on the field, relative to the robot)
   */
  const virtual Vector2f getPosition() = 0;

  /**
   * Every instance of a filter has to implement this function for getting the ball estimate position uncertainty
   * @return The filter covariance reading the position estimate
   */
  const virtual Matrix2f getPositionCovariance() = 0;

  /**
   * Every instance of a filter can implement this function for getting the ball estimate velocity
   * @return The velocity of the ball (on the field, relative to the robot), (0,0) by default
   */
  const virtual Vector2f getVelocity() { return Vector2f::Zero(); }
};

/**
 * State estimation for a stationary / lying ball.
 * Based on a standard Kalman filter
 */
class StationaryBallKalmanFilter : public BallStateEstimate
{
public:
  Vector2f x = Vector2f::Zero();       /**< Mean of the estimated stationary ball */
  Matrix2f P = Matrix2f::Identity();   /**< Covariance matrix of the ball estimate */

  /** Returns the mean of the internal estimate
    * @return Yes, exactly.
    */
  const Vector2f getPosition(){ return x; }

  /** Returns the covariance of the internal estimate
   * @return Yes, exactly.
   */
  const Matrix2f getPositionCovariance() { return P; }

  /** Moves hypothesis and updates mean and covariance accordingly */
  void motionUpdate(const Vector4f& squaredProcessCov,
                    const Vector4f& odometryTranslationCov,
                    const Matrix2f& fixedOdometryRotation, const Vector2f& fixedOdometryTranslation,
                    const Matrix2f& fixedOdometryRotationTransposed, const Matrix2f& fixedOdometryRotationDeviationRotation)
  {
    // save old position
    lastPosition = x;

    // predict
    x = fixedOdometryRotation * x + fixedOdometryTranslation;
    P = fixedOdometryRotation * P * fixedOdometryRotationTransposed;

    // add process noise
    P(0, 0) += squaredProcessCov[0];
    P(1, 1) += squaredProcessCov[1];

    // add odometry translation noise
    P(0, 0) += odometryTranslationCov[0];
    P(1, 1) += odometryTranslationCov[1];

    // add noise from odometry rotation (crude approximation)
    Vector2f squaredOdometryRotationDeviationTranslation = (fixedOdometryRotationDeviationRotation * x - x).cwiseAbs2();
    P(0, 0) += squaredOdometryRotationDeviationTranslation.x();
    P(1, 1) += squaredOdometryRotationDeviationTranslation.y();
  }

  /** Kalman filter measurement update step
   *  @param measurement The ball perception in relative field coordinates
   *  @param measurementCov The covariance of the measurement
   */
  void integrateMeasurement(const Vector2f& measurement, const Matrix2f& measurementCov)
  {
    Matrix2f covPlusSensorCov = P;
    covPlusSensorCov += measurementCov;
    const Matrix2f K = P* covPlusSensorCov.inverse();
    const Vector2f innovation = measurement - x;
    const Vector2f correction = K * innovation;
    x += correction;
    P -= K * P;
  }
};

/**
 * State estimation for a rolling ball.
 * Based on a standard Kalman filter
 */
class RollingBallKalmanFilter : public BallStateEstimate
{
public:
  Vector4f x = Vector4f::Zero();           /**< Mean of the estimated rolling ball */
  Matrix4f P = Matrix4f::Identity();       /**< Covariance matrix of the ball estimate */
  Matrix2x4f H   = Matrix2x4f::Identity(); /**< Measurement/observation matrix for relating the dimensionality of the measurements (2) to the dimensionality of the state estimate (4) */
  Matrix4x2f H_T = H.transpose();          /**< Transposed measurement matrix */

  /** Empty default constructor */
  RollingBallKalmanFilter() {}

  /** Constructor based on stationary filter, useful when a ball is kicked and becomes converted
   *   @param sbkf The stationary filter
   *   @param newPosition The position (which might have slightly changed after kick)
   *   @param newVelocity The velocity
   *   @param addVelocityCov Covariance of velocity
   */
  RollingBallKalmanFilter(const StationaryBallKalmanFilter& sbkf,
                          const Vector2f& newPosition,
                          const Vector2f& newVelocity,
                          const Vector2f& addVelocityCov)
  {
    x << newPosition, newVelocity;
    lastPosition = sbkf.x;
    P = Matrix4f::Identity();
    P.topLeftCorner(2, 2) << sbkf.P;
    P(2, 2) += addVelocityCov.x();
    P(3, 3) += addVelocityCov.y();
    radius = sbkf.radius;
    numOfMeasurements = sbkf.numOfMeasurements;
    likelihoodOfMeasurements = sbkf.likelihoodOfMeasurements;
    weighting = sbkf.weighting;
  }

  /** Returns the upper part of the mean of the internal estimate
   * @return Yes, exactly.
   */
  const Vector2f getPosition() { return x.topRows(2); }

  /** Returns the covariance of the internal estimate
   * @return Yes, exactly.
   */
  const Matrix2f getPositionCovariance() { return P.topLeftCorner(2,2); }

  /** Returns the lower part of the mean of the internal estimate
   * @return Yes, exactly.
   */
  const Vector2f getVelocity() { return x.bottomRows(2); }

  /** Moves hypothesis and updates mean and covariance accordingly */
  void motionUpdate(const Matrix4f& movingA, const Matrix4f& movingATransposed,
                    const Vector4f& movingOdometryTranslation, const Vector4f& squaredProcessCov,
                    const Vector4f& odometryTranslationCov, const Matrix4f& movingOdometryRotationDeviationRotation,
                    float friction, float deltaTime)
  {
    // save old position
    lastPosition = x.topRows(2);;

    // predict
    x = movingA * x + movingOdometryTranslation;
    P = movingA * P * movingATransposed;

    // add process noise
    for(int i = 0; i < 4; ++i)
      P(i, i) += squaredProcessCov[i];

    // add odometry translation noise
    for(int i = 0; i < 4; ++i)
      P(i, i) += odometryTranslationCov[i];

    // add noise from odometry rotation (crude approximation)
    Vector4f squaredOdometryRotationDeviationTranslation = (movingOdometryRotationDeviationRotation * x - x).cwiseAbs2();
    for(int i = 0; i < 4; ++i)
      P(i, i) += squaredOdometryRotationDeviationTranslation[i];

    // add friction
    Vector2f newPosition = x.topRows(2);
    Vector2f newVelocity = x.bottomRows(2);
    BallPhysics::applyFrictionToPositionAndVelocity(newPosition, newVelocity, deltaTime, friction);
    x << newPosition, newVelocity;
  }

  /** Kalman filter measurement update step
   *  @param measurement The ball perception in relative field coordinates
   *  @param measurementCov The covariance of the measurement
   */
  void integrateMeasurement(const Vector2f& measurement, const Matrix2f& measurementCov)
  {
    Matrix2f covPlusSensorCov = H * P * H_T;
    covPlusSensorCov += measurementCov;
    const Matrix4x2f K = P * H_T * covPlusSensorCov.inverse();
    const Vector2f innovation = measurement - H * x;
    const Vector4f correction = K * innovation;
    x += correction;
    P -= K * H * P;
  }

  /** Converts to a stationary filter, useful when ball stops
   *  @return A stationary filter that contains all relevant information from this filter.
   */
  const StationaryBallKalmanFilter toStationaryBallKalmanFilter()
  {
    StationaryBallKalmanFilter sbkf;
    sbkf.x = x.topRows(2);
    sbkf.lastPosition = sbkf.x;
    sbkf.P = P.topLeftCorner(2, 2);
    sbkf.radius = radius;
    sbkf.numOfMeasurements = numOfMeasurements;
    sbkf.likelihoodOfMeasurements = likelihoodOfMeasurements;
    sbkf.weighting = weighting;
    return sbkf;
  }
};
