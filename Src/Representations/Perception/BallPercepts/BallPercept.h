/**
 * @file BallPercept.h
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @autor Jesse
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include <vector>

STREAMABLE(BallPercept,
{
  ENUM(Status,
  {,
    notSeen, /**< The ball was not seen. */
    seen,    /**< The ball was seen. */
    guessed, /**< Would be ok for a moving ball. */
  });

  BallPercept() = default;
  inline BallPercept(const Vector2f& positionInImage, const float radiusInImage, const Vector2f& relativePositionOnField, const float radiusOnField, const BallPercept::Status status);

  /** Draws the ball*/
  void draw() const;

  /** Verifies that the ball percept contains valid values. */
  void verify() const,

  (Vector2f) positionInImage,         /**< The position of the ball in the current image */
  (float) radiusInImage,              /**< The radius of the ball in the current image */
  (Status)(notSeen) status,           /**< Indicates, if the ball was seen in the current image. */
  (Vector2f) positionOnField,         /**< Ball position relative to the robot. */
  (float)(50.f) radiusOnField,        /**< The radius of the ball on the field in mm */
});

inline BallPercept::BallPercept(const Vector2f& positionInImage, const float radiusInImage, const Vector2f& positionOnField, const float radiusOnField, const BallPercept::Status status = BallPercept::Status::seen) :
  positionInImage(positionInImage), radiusInImage(radiusInImage), status(status), positionOnField(positionOnField), radiusOnField(radiusOnField) {}
