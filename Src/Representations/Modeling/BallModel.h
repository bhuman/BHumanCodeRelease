/**
* @file BallModel.h
*
* Declaration of class BallModel
*
* @author <A href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</A>
*/

#pragma once

#include "Tools/Math/Matrix2x2.h"
#include "Tools/Streams/AutoStreamable.h"

class Pose2D;

/**
 * @class BallState
 *
 * Base class for ball position and velocity.
 */
STREAMABLE(BallState,
{
public:
  /**
  * Computes the position were a rolling ball is expected to stop rolling.
  * @return The position relative to the robot (in mm)
  */
  Vector2<> getEndPosition(float ballFriction) const
  {
    return position + velocity * (-1 / ballFriction);
  }

  /**
  * Computes the distance the ball will still be rolling
  */
  float distanceToEndPosition(float ballFriction) const
  {
    return (position-getEndPosition(ballFriction)).abs();
  }

  /**
  * Computes the time (in seconds) the ball needs to pass distance.
  * Return std::numeric_limits<float>::max(), if ball won't make the distance with its current velocity.
  */
  float timeForDistance(float distance, float ballFriction) const
  {
    if(distanceToEndPosition(ballFriction) < distance)
      return std::numeric_limits<float>::max();
    else
      return std::log(distance * ballFriction / velocity.abs() + 1) / ballFriction;
  },

  (Vector2<>) position,               /**< The position of the ball relative to the robot (in mm)*/
  (Vector2<>) velocity,               /**< The velocity of the ball relative to the robot (in mm/s)*/
  (Matrix2x2<>) positionCovariance,   /**< The covariance matrix describing the position uncertainty */
});

/**
 * @class BallModel
 *
 * Contains all current knowledge about the ball.
 */
STREAMABLE(BallModel,
{
public:
  /** Draws the estimate on the field */
  void draw() const;

  /** Draws the end position of the estimate on the field */
  void drawEndPosition(float ballFriction) const;

  /** Draw the ball model in scene */
  void draw3D(const Pose2D& robotPose) const,

  (Vector2<>) lastPerception, /**< The last seen position of the ball */
  (BallState) estimate, /**< The state of the ball estimated from own observations; it is propagated even if the ball is not seen */
  (unsigned)(0) timeWhenLastSeen, /**< Time stamp, indicating what its name says */
  (unsigned)(0) timeWhenDisappeared, /**< The time when the ball was not seen in the image altough it should have been there */
});


/**
* @class GroundTruthBallModel
* The same as the BallModel, but - in general - provided by an external
* source that has ground truth quality
*/
class GroundTruthBallModel : public BallModel
{
public:
  /** Draws something*/
  void draw() const;
};


/**
* @class BallModelCompressed
* A compressed version of BallModel used in team communication
*/
STREAMABLE(BallModelCompressed,
{
public:
  BallModelCompressed(const BallModel& ballModel);
  operator BallModel() const,

  (Vector2<short>) lastPerception, /**< The last seen position of the ball */
  (Vector2<short>) position,
  (Vector2<short>) velocity,
  (unsigned) timeWhenLastSeen, /**< Time stamp, indicating what its name says */
  (unsigned) timeWhenDisappeared, /**< The time when the ball was not seen in the image altough it should have been there */
});
