/**
 * @file BallModel.h
 *
 * Declaration of struct BallModel
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Communication/BHumanMessageParticle.h"
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct BallState
 *
 * Base struct for ball position and velocity.
 */
STREAMABLE(BallState,
{,
  (Vector2f)(Vector2f::Zero()) position,       /**< The position of the ball relative to the robot (in mm)*/
  (Vector2f)(Vector2f::Zero()) velocity,       /**< The velocity of the ball relative to the robot (in mm/s)*/
  (float)(50) radius,                          /**< The assumed radius of the ball (in mm)*/
  (Matrix2f)(Matrix2f::Identity()) covariance, /**< The covariance matrix of the ball position*/
});

/**
 * @struct BallModel
 *
 * Contains all current knowledge about the ball.
 */
STREAMABLE(BallModel, COMMA public BHumanCompressedMessageParticle<BallModel>
{
  /** Verifies that the ball model contains valid values. */
  void verify() const;
  /** Draws the estimate on the field */
  void draw() const,

  (Vector2f)(Vector2f::Zero()) lastPerception, /**< The last seen position of the ball */
  (BallState) estimate, /**< The state of the ball estimated from own observations; it is propagated even if the ball is not seen */
  (unsigned)(0) timeWhenLastSeen,    /**< Time stamp, indicating what its name says */
  (unsigned)(0) timeWhenDisappeared, /**< The time when the ball was not seen in the image although it should have been there */
  (unsigned)(0) timeOfLastCollision, /**< The last point of time when the ball model computation incorporated a collision between the ball and the robot */
  (unsigned char)(0) seenPercentage, /**< How often was the ball seen in the recent past (0%...100%). */
  (BallState) riskyMovingEstimate, /**< Do not use this, unless you are REALLY knowing what you are doing. If the normal estimate contains a stationary ball, this element might contain the most likely hypothesis for a rolling ball. Do not use this, unless you are REALLY knowing what you are doing. Do not ... */
  (bool)(false) riskyMovingEstimateIsValid, /**< If set to true, riskyMovingEstimate contains some data. */
});

/**
 * @struct GroundTruthBallModel
 * The same as the BallModel, but - in general - provided by an external
 * source that has ground truth quality
 */
STREAMABLE_WITH_BASE(GroundTruthBallModel, BallModel,
{
  /** Draws something*/
  void draw() const,
});
