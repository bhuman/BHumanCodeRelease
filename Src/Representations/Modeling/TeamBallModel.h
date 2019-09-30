/**
 * @file TeamBallModel.h
 *
 * Declaration of a representation that represents a ball model based
 * on own observations as well as on teammate observations.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct TeamBallModel
 */
STREAMABLE(TeamBallModel,
{
  ENUM(Contributors,
  {,
    onlyMe,         /**< This is just my own ball model. Go, team, go ??!!? :-( */
    oneOther,       /**< The ball model of one other robot was used. */
    multipleOthers, /**< The ball models of multiple other robots have been combined. */
    meAndOthers,    /**< My ball was combined which the ball model(s) of others. Teamwork! :-) */
  });

  void verify() const;
  void draw() const,

  (Vector2f) position,                         /**< The position of the ball in global field coordinates (in mm) */
  (Vector2f) velocity,                         /**< The velocity of the ball in global field coordinates (in mm/s) */
  (bool)(false) isValid,                       /**< Position and velocity are valid (i.e. somebody has seen the ball), if true */
  (unsigned)(0) timeWhenLastValid,             /**< Workaround by Andreas St. -> Tim L. suggests to remove it */
  (unsigned)(0) timeWhenLastSeen,              /**< The point of time when the ball was seen the last time by a teammate */
  (Contributors)(onlyMe) contributors,         /**< Information about which robot(s) contributed to the computation of this model */
  (std::vector<Vector2f>) balls,               /**< List of all merged ball positions */
});
