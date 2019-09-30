/**
 * @file BallContactChecker.h
 *
 * This file defines a representation that allows to check, if one of the
 * robot's feet currently touches (and thus moves!) the ball.
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"


/**
 * @struct BallContactInformation
 */
STREAMABLE(BallContactInformation,
{
  /** Details about the robot part(s) that touched the ball */
  ENUM(ContactType,
       {,
         leftFoot,    /**< Exactly! */
         rightFoot,   /**< You guessed it! */
         center,      /**< Robot makes a blocking motion */
         none,        /**< No ball contact (for initialization) */
       });
  ,
  (ContactType)(none) contactType,               /**< Which part of the robot touched the ball */
  (Vector2f)(Vector2f::Zero())  newPosition,     /**< The position of the ball (after the contact) relative to the robot (in mm) */
  (Vector2f)(Vector2f::Zero())  newVelocity,     /**< The velocity of the ball (after the contact) relative to the robot (in mm/s) */
  (Vector2f)(Vector2f(1.f,1.f)) addVelocityCov,  /**< The amount of uncertainty that is added to the velocity after the contact */
});


STREAMABLE(BallContactChecker,
{
  /**
   * The function needs some comments
   */
  FUNCTION(bool(const Vector2f& p, const Vector2f& v, const Vector2f& lastP, BallContactInformation& contactInfo)) collide,
});
