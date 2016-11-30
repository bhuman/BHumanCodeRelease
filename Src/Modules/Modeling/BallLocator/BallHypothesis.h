/**
 * @file BallHypothesis.h
 * Ball state estimation
 * @author Colin Graf
 * @author Tim Laue
 */

#pragma once

#include "BallLocatorTools.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Modeling/BallPhysics.h"

struct BallHypothesis
{
  ENUM(Type,
  {,
    moving,
    stationary,
  });

  Type type;
  float gain;
  float weight;
  float height;
  int age;

  float radius;

  Vector4f movingX = Vector4f::Zero();
  Matrix4f movingCov = Matrix4f::Identity();

  Vector2f stationaryX = Vector2f::Zero();
  Matrix2f stationaryCov = Matrix2f::Identity();

  void motionUpdate(BallModel& ballModel, const Matrix4f& movingA, const Matrix4f& movingATransposed,
                    const Vector4f& movingOdometryTranslation, const Vector4f& squaredProcessCov,
                    const Vector4f& odometryTranslationCov, const Matrix4f& movingOdometryRotationDeviationRotation,
                    const Matrix2f& fixedOdometryRotation, const Vector2f& fixedOdometryTranslation,
                    const Matrix2f& fixedOdometryRotationTransposed, const Matrix2f& fixedOdometryRotationDeviationRotation,
                    const FieldDimensions& theFieldDimensions, float deltaTime)
  {
    if(type == BallHypothesis::moving)
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
    if(type == BallHypothesis::moving)
    {
      Vector4f& x = movingX;
      Vector2f newPosition = x.topRows(2);
      Vector2f newVelocity = x.bottomRows(2);
      BallPhysics::applyFrictionToPositionAndVelocity(newPosition, newVelocity, deltaTime, theFieldDimensions.ballFriction);
      x << newPosition, newVelocity;
      if(newVelocity.squaredNorm() < sqr(100.f))
      {
        // the estimate stopped moving
        type = BallHypothesis::stationary;
        stationaryX = x.topRows(2);
        Matrix4f& cov = movingCov;
        stationaryCov << cov.topLeftCorner(2, 2);
      }
    }
  }

  void sensorUpdate(const Vector2f& measurement, const Matrix2f& measurementCov,
                    const Matrix2x4f& c, const Matrix4x2f& cTransposed,
                    const BallPercept& theBallPercept)
  {
    if(type == BallHypothesis::moving)
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
      age++;

      Matrix2f covPlusSensorCov = c * cov * cTransposed;
      covPlusSensorCov += measurementCov;
      Matrix4x2f k = cov * cTransposed * covPlusSensorCov.inverse();
      Vector2f innovation = measurement - c * x;
      Vector4f correction = k * innovation;
      x += correction;
      cov -= k * c * cov;
    }
    else // type == State::stationary
    {
      Vector2f& x = stationaryX;
      Matrix2f& cov = stationaryCov;

      ASSERT(weight >= 0.f);
      gain = BallLocatorTools::getUnscaledProbabilityAt(measurement, measurementCov, x);

      if(!(gain >= 0.f && gain <= 10.f))
      {
        OUTPUT_ERROR("getUnscaledProbabilityAt: gain=" << gain);
        OUTPUT_ERROR("measurement x=" << measurement.x() << " y=" << measurement.y());
        OUTPUT_ERROR("measurementCov r0c0=" << measurementCov(0, 0) << " r0c1=" << measurementCov(0, 1) << " r1c0=" << measurementCov(1, 0) << " r1c1=" << measurementCov(1, 1));
        OUTPUT_ERROR("x x.x=" << x[0] << " x.y=" << x[1]);
      }

      ASSERT(gain >= 0.f && gain <= 10.f);
      weight *= gain;
      height = weight * BallLocatorTools::getProbabilityAtMean(cov);
      age++;

      Matrix2f covPlusSensorCov = cov;
      covPlusSensorCov += measurementCov;
      Matrix2f k = cov * covPlusSensorCov.inverse();
      Vector2f innovation = measurement - x;
      Vector2f correction = k * innovation;
      x += correction;
      cov -= k * cov;
    }
    radius = theBallPercept.radiusOnField;
  }
};
