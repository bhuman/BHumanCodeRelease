/**
 * @file KalmanFilterBallHypothesis.h
 *
 * State estimation for a rolling or stationary ball, based
 * on a standard Kalman filter
 *
 * @author Colin Graf
 * @author Tim Laue
 */

#pragma once

#include "Tools/Debugging/Debugging.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Modeling/BallLocatorTools.h"

/**
 * State estimation for a rolling or stationary ball, based
 * on a standard Kalman filter
 */
struct KalmanFilterBallHypothesis
{
  /**< Type for distinguishing the ball status */
  ENUM(Type,
  {,
    moving,
    stationary,
  });

  Type type;             /**< Member that stores the type (moving / stationary) */
  float weight;          /**< Kind of likelihood of hypothesis (based on probabilities of measurements) */
  float height;          /**< Kind of likelihood of hypothesis (weight multiplied with probability at mean) */
  float radius;          /**< Last perceived radius of the ball */
  float gain;            /**< Probability at mean of current measurement */
  int numOfMeasurements; /**< The number of measurements that have been fused in this hypothesis */

  Vector4f movingX = Vector4f::Zero();             /** Mean of estimated rolling ball */
  Matrix4f movingCov = Matrix4f::Identity();       /** Covariance of estimated rolling ball */
  Vector2f stationaryX = Vector2f::Zero();         /** Mean of estimated stationary ball */
  Matrix2f stationaryCov = Matrix2f::Identity();   /** Covariance of estimated stationary ball */
  
  const Matrix2x4f c = Matrix2x4f::Identity();
  const Matrix4x2f cTransposed = c.transpose();

  /** Moves hypothesis and updates mean and covariance accordingly */
  void motionUpdate(const Matrix4f& movingA, const Matrix4f& movingATransposed,
                    const Vector4f& movingOdometryTranslation, const Vector4f& squaredProcessCov,
                    const Vector4f& odometryTranslationCov, const Matrix4f& movingOdometryRotationDeviationRotation,
                    const Matrix2f& fixedOdometryRotation, const Vector2f& fixedOdometryTranslation,
                    const Matrix2f& fixedOdometryRotationTransposed, const Matrix2f& fixedOdometryRotationDeviationRotation,
                    float friction, float deltaTime)
  {
    if(type == moving)
    {
      Vector4f& x = movingX;
      Matrix4f& cov = movingCov;

      // predict
      x = movingA * x + movingOdometryTranslation;
      cov = movingA * cov * movingATransposed;

      // add process noise
      for(int i = 0; i < 4; ++i)
        cov(i, i) += squaredProcessCov[i];

      // add odometry translation noise
      for(int i = 0; i < 4; ++i)
        cov(i, i) += odometryTranslationCov[i];

      // add noise from odometry rotation (crude approximation)
      Vector4f squaredOdometryRotationDeviationTranslation = (movingOdometryRotationDeviationRotation * x - x).cwiseAbs2();
      for(int i = 0; i < 4; ++i)
        cov(i, i) += squaredOdometryRotationDeviationTranslation[i];
    }
    else // state->type == State::stationary
    {
      Vector2f& x = stationaryX;
      Matrix2f& cov = stationaryCov;

      // predict
      x = fixedOdometryRotation * x + fixedOdometryTranslation;
      cov = fixedOdometryRotation * cov * fixedOdometryRotationTransposed;

      // add process noise
      cov(0, 0) += squaredProcessCov[0];
      cov(1, 1) += squaredProcessCov[1];

      // add odometry translation noise
      cov(0, 0) += odometryTranslationCov[0];
      cov(1, 1) += odometryTranslationCov[1];

      // add noise from odometry rotation (crude approximation)
      Vector2f squaredOdometryRotationDeviationTranslation = (fixedOdometryRotationDeviationRotation * x - x).cwiseAbs2();
      cov(0, 0) += squaredOdometryRotationDeviationTranslation.x();
      cov(1, 1) += squaredOdometryRotationDeviationTranslation.y();
    }
    // apply friction
    if(type == moving)
    {
      Vector4f& x = movingX;
      Vector2f newPosition = x.topRows(2);
      Vector2f newVelocity = x.bottomRows(2);
      BallPhysics::applyFrictionToPositionAndVelocity(newPosition, newVelocity, deltaTime, friction);
      x << newPosition, newVelocity;
      if(newVelocity.squaredNorm() < sqr(100.f))
      {
        // the estimate stopped moving
        type = stationary;
        stationaryX = x.topRows(2);
        Matrix4f& cov = movingCov;
        stationaryCov << cov.topLeftCorner(2, 2);
      }
    }
  }

  /** Updates hypothesis based on a new measurement
   *  and thus updates mean and covariance as well as weight, height, and radius
   *  @param measurement The ball perception in relative field coordinates
   *  @param measurementCov The covariance of the measurement
   *  @param radiusOnField The ball radius, just copied to member variable
   */
  void sensorUpdate(const Vector2f& measurement, const Matrix2f& measurementCov, float radiusOnField)
  {
    if(type == moving)
    {
      Vector4f& x = movingX;
      Matrix4f& cov = movingCov;

      ASSERT(weight >= 0.f);
      gain = BallLocatorTools::getUnscaledProbabilityAt(measurement, measurementCov, x.topRows(2));
      if(!(gain >= 0.f && gain <= 10.f))
      {
        OUTPUT_ERROR("getUnscaledProbabilityAt: gain=" << gain);
        OUTPUT_ERROR("measurement x=" << measurement.x() << " y=" << measurement.y());
        OUTPUT_ERROR("measurementCov r0c0=" << measurementCov(0, 0) << " r0c1=" << measurementCov(0, 1) << " r1c0=" << measurementCov(1, 0) << " r1c1=" << measurementCov(1, 1));
        OUTPUT_ERROR("x x.x=" << x[0] << " x.y=" << x[1]);
      }
      ASSERT(gain >= 0.f && gain <= 10.f);
      weight *= gain;
      height = weight * BallLocatorTools::getProbabilityAtMean(cov.topLeftCorner(2, 2));

      Matrix2f covPlusSensorCov = c * cov * cTransposed;
      covPlusSensorCov += measurementCov;
      Matrix4x2f k = cov * cTransposed * covPlusSensorCov.inverse();
      Vector2f innovation = measurement - c * x;
      Vector4f correction = k * innovation;
      x += correction;
      cov -= k * c * cov;
    }
    else // type == stationary
    {
      Vector2f& x = stationaryX;
      Matrix2f& cov = stationaryCov;

      ASSERT(weight >= 0.f);
      gain = BallLocatorTools::getUnscaledProbabilityAt(measurement, measurementCov, x);

      if(!(gain >= 0.f && gain <= 10.f))
      {
        FAIL("KalmanFilterBallHypothesis:gain was out of bounds: " << gain <<
             "  measurement x=" << measurement.x() << "  y=" << measurement.y() <<
             "  measurementCov r0c0=" << measurementCov(0, 0) << " r0c1=" << measurementCov(0, 1) <<
             " r1c0=" << measurementCov(1, 0) << " r1c1=" << measurementCov(1, 1) <<
             "  State x.x=" << x[0] << " x.y=" << x[1]);
      }

      weight *= gain;
      height = weight * BallLocatorTools::getProbabilityAtMean(cov);

      Matrix2f covPlusSensorCov = cov;
      covPlusSensorCov += measurementCov;
      Matrix2f k = cov * covPlusSensorCov.inverse();
      Vector2f innovation = measurement - x;
      Vector2f correction = k * innovation;
      x += correction;
      cov -= k * cov;
    }
    radius = radiusOnField;
    numOfMeasurements++;
  }
};
