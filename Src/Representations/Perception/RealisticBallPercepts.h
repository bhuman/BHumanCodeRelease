/**
 * @file RealisticBallPercepts.h
 *
 * Representation of the balls seen in the current frame
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(RealisticBallPercept,
{,
  (Vector2f) positionInImage,         /**< The position of the ball in the current image */
  (float) radiusInImage,              /**< The radius of the ball in the current image */
  (Vector2f) relativePositionOnField, /**< Ball position relative to the robot. */
  (Vector2f) absolutePositionOnField, /**< Ball position relative to the center of the field. */
  (float) radiusOnField,              /**< The radius of the ball on the field in mm */
});

STREAMABLE(RealisticBallPercepts,
{
  void draw() const,

  (std::vector<RealisticBallPercept>) balls, /**< Balls perceived in the current image */
});
