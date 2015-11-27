/**
 * @file BallPercept.h
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(BallPercept,
{
  ENUM(Status,
  {,
    notSeen,
    seen,
    checkNoNoise,
    checkBallSpot,
    searchBallPoints,
    checkBallPoints,
    calculateBallInImage,
    checkBallInImage,
    calculateBallOnField,
    checkBallOnField,
    checkJersey,
  });

  /** Draws the ball*/
  void draw() const,

  (Vector2f) positionInImage,         /**< The position of the ball in the current image */
  (float) radiusInImage,              /**< The radius of the ball in the current image */
  (Status)(notSeen) status,           /**< Indicates, if the ball was seen in the current image. */
  (Vector2f) relativePositionOnField, /**< Ball position relative to the robot. */
  (float)(35) radiusOnField,          /**< The radius of the ball on the field in mm */
});
