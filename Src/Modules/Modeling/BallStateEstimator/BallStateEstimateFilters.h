/**
 * @file BallStateEstimateFilters.h
 *
 * Different state estimators for rolling and stationary balls.
 * Currently based on a standard Kalman filters
 *
 * @author Tim Laue
 */

#pragma once

#include "Debugging/Debugging.h"
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
  float radius = 1.f;                                     /**< Last perceived radius of the ball */
  int numOfMeasurements = 0;                              /**< The number of measurements that have been fused in this hypothesis */
  float nllOfMeasurements = 0.f;                          /**< Sum of negative log-likelihoods (NLLs) of integrated measurements, shifted in relation to hypothesis with lowest value (which then has a value of 0) */
  float nllWeighting = std::numeric_limits<float>::max(); /**< Kind of NLL of hypothesis (NLL of measurements plus NLL at mean of covariance) */
  Vector2f lastPosition = Vector2f::Zero();               /**< Store position before update, needed for collision detection */
  unsigned timeOfLastCollision = 0;                       /**< The last point of time when the estimate computation incorporated a collision with the robot */

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
    const Vector2f x = getPosition();
    const Matrix2f P = getPositionCovariance();
    const float gain = BallLocatorTools::getNLLOfPosition(measurement, measurementCov, x);
    ASSERT(std::isfinite(gain));
    nllOfMeasurements += gain;
    nllWeighting = nllOfMeasurements + BallLocatorTools::getNLLOfMean(P);

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
  const virtual Vector2f getPosition() const = 0;

  /**
   * Every instance of a filter has to implement this function for getting the ball estimate position uncertainty
   * @return The filter covariance reading the position estimate
   */
  const virtual Matrix2f getPositionCovariance() const = 0;

  /**
   * Every instance of a filter can implement this function for getting the ball estimate velocity
   * @return The velocity of the ball (on the field, relative to the robot), (0,0) by default
   */
  const virtual Vector2f getVelocity() const { return Vector2f::Zero(); }
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
  const Vector2f getPosition() const { return x; }

  /** Returns the covariance of the internal estimate
   * @return Yes, exactly.
   */
  const Matrix2f getPositionCovariance() const { return P; }

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
    P.diagonal() += squaredProcessCov.head<2>();

    // add odometry translation noise
    P.diagonal() += odometryTranslationCov.head<2>();

    // add noise from odometry rotation (crude approximation)
    const Vector2f squaredOdometryRotationDeviationTranslation = (fixedOdometryRotationDeviationRotation * x - x).cwiseAbs2();
    P.diagonal() += squaredOdometryRotationDeviationTranslation;
  }

  /** Kalman filter measurement update step
   *  @param measurement The ball perception in relative field coordinates
   *  @param measurementCov The covariance of the measurement
   */
  void integrateMeasurement(const Vector2f& measurement, const Matrix2f& measurementCov)
  {
    Matrix2f covPlusSensorCov = P;
    covPlusSensorCov += measurementCov;
    const Matrix2f K = P * covPlusSensorCov.inverse();
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

  /** Empty default constructor */
  RollingBallKalmanFilter() = default;

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
    P = Matrix4f::Identity();
    P.topLeftCorner<2, 2>() << sbkf.P;
    P(2, 2) += addVelocityCov.x();
    P(3, 3) += addVelocityCov.y();
    lastPosition = sbkf.x;
    radius = sbkf.radius;
    numOfMeasurements = sbkf.numOfMeasurements;
    nllOfMeasurements = sbkf.nllOfMeasurements;
    nllWeighting = sbkf.nllWeighting;
  }

  /** Returns the upper part of the mean of the internal estimate
   * @return Yes, exactly.
   */
  const Vector2f getPosition() const { return x.topRows<2>(); }

  /** Returns the covariance of the internal estimate
   * @return Yes, exactly.
   */
  const Matrix2f getPositionCovariance() const { return P.topLeftCorner<2, 2>(); }

  /** Returns the lower part of the mean of the internal estimate
   * @return Yes, exactly.
   */
  const Vector2f getVelocity() const { return x.bottomRows(2); }

  /** Moves hypothesis and updates mean and covariance accordingly */
  void motionUpdate(const Matrix4f& movingA, const Matrix4f& movingATransposed,
                    const Vector4f& movingOdometryTranslation, const Vector4f& squaredProcessCov,
                    const Vector4f& odometryTranslationCov, const Matrix4f& movingOdometryRotationDeviationRotation,
                    float friction, float deltaTime)
  {
    // save old position
    lastPosition = x.topRows<2>();

    // predict
    x = movingA * x + movingOdometryTranslation;
    P = movingA * P * movingATransposed;

    // add process noise
    P.diagonal() += squaredProcessCov;

    // add odometry translation noise
    P.diagonal() += odometryTranslationCov;

    // add noise from odometry rotation (crude approximation)
    const Vector4f squaredOdometryRotationDeviationTranslation = (movingOdometryRotationDeviationRotation * x - x).cwiseAbs2();
    P.diagonal() += squaredOdometryRotationDeviationTranslation;

    // add friction
    Vector2f newPosition = x.topRows<2>();
    Vector2f newVelocity = x.bottomRows<2>();
    BallPhysics::applyFrictionToPositionAndVelocity(newPosition, newVelocity, deltaTime, friction);
    x << newPosition, newVelocity;
  }

  /** Kalman filter measurement update step
   *  @param measurement The ball perception in relative field coordinates
   *  @param measurementCov The covariance of the measurement
   */
  void integrateMeasurement(const Vector2f& measurement, const Matrix2f& measurementCov)
  {
    Matrix2f covPlusSensorCov = P.topLeftCorner<2, 2>();
    covPlusSensorCov += measurementCov;
    const Matrix4x2f K = P.leftCols<2>() * covPlusSensorCov.inverse();
    const Vector2f innovation = measurement - x.topRows<2>();
    const Vector4f correction = K * innovation;
    x += correction;
    P -= K * P.topRows<2>();
  }

  /** Converts to a stationary filter, useful when ball stops
   *  @return A stationary filter that contains all relevant information from this filter.
   */
  const StationaryBallKalmanFilter toStationaryBallKalmanFilter()
  {
    StationaryBallKalmanFilter sbkf;
    // Marginalize over the velocity.
    sbkf.x = x.topRows<2>();
    sbkf.P = P.topLeftCorner<2, 2>();
    sbkf.lastPosition = sbkf.x;
    sbkf.radius = radius;
    sbkf.numOfMeasurements = numOfMeasurements;
    sbkf.nllOfMeasurements = nllOfMeasurements;
    sbkf.nllWeighting = nllWeighting;
    return sbkf;
  }
};
