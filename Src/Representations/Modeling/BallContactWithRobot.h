/**
 * @file BallContactWithRobot.h
 *
 * Declaration of a representation that represents information about
 * the last contact of the robot and the ball.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct BallContactWithRobot
 */
STREAMABLE(BallContactWithRobot,
{
  /** Details about the robot part(s) that touched the ball */
  ENUM(ContactType,
  {,
    leftFoot,    /**< Exactly! */
    rightFoot,   /**< You guessed it! */
    center,      /**< Robot makes a blocking motion */
    none,        /**< No ball contact (for initialization) */
  });

  /** Makes a drawing */
  void draw() const;

  /** Verifies vality of values */
  void verify() const,

  (unsigned)(0) timeOfLastContact,               /**< Time stamp that holds the point of time of the last contact */
  (ContactType)(none) contactType,               /**< Which part of the robot touched the ball */
  (Vector2f)(Vector2f::Zero())  newPosition,     /**< The position of the ball (after the contact) relative to the robot (in mm) */
  (Vector2f)(Vector2f::Zero())  newVelocity,     /**< The velocity of the ball (after the contact) relative to the robot (in mm/s) */
  (Vector2f)(Vector2f(1.f,1.f)) addVelocityCov,  /**< The amount of uncertainty that is added to the velocity after the contact */
});
