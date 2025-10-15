/**
 * @file TeamBallModel.h
 *
 * Declaration of a representation that represents a ball model based
 * on my observations and observations made by my teammates.
 * The goal is that the TeamBall is the same for all players
 * Issue #1318
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 * @contributor Liam Hurwitz
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include "Math/Eigen.h"

/**
 * @struct TeamBallModel
 */
STREAMABLE(TeamBallModel,
{
  void verify() const;
  void draw() const,

  (Vector2f) position,                         /**< The position of the ball in global field coordinates (in mm) */
  (Vector2f) velocity,                         /**< The velocity of the ball in global field coordinates (in mm/s) */
  (bool)(false) isValid,                       /**< Position and velocity are valid (i.e. somebody has seen the ball), if true */
  (bool)(false) newerThanOwnBall,              /**< true if a teammate has more recently seen and communicated a ball than we locally have*/
});
