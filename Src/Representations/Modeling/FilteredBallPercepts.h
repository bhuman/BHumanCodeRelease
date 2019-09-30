/**
 * @file FilteredBallPercepts.h
 *
 * Declaration of struct FilteredBallPercepts, a representation that
 * holds information about really seen balls and does (hopefully) not
 * contain any false positives ;-)
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

/**
 * @struct FilteredBallPercept
 *
 * A single ball perception, similar to the BallPercept but
 * having a timestamp instead of a status and a few less elements.
 */
STREAMABLE(FilteredBallPercept,
{
  /**< Default constructor */
  FilteredBallPercept() = default;

  /** Constructor with field initialization
   * @param posImg The ball position in the image
   * @param posFld The ball position on the field (relative to the robot)
   * @param covFld The covariance of the measurement (relative to the robot)
   * @param r The ball radius on the field
   * @param t The point of time when this ball was seen
   */
  FilteredBallPercept(const Vector2f& posImg, const Vector2f& posFld, const Matrix2f& covFld, float r, unsigned t);
  ,
  (Vector2f) positionInImage,         /**< The position of the ball in the current image */
  (Vector2f) positionOnField,         /**< Ball position relative to the robot. */
  (Matrix2f) covOnField,              /**< Measurement covariance on field. */
  (float)(50.f) radiusOnField,        /**< The radius of the ball on the field in mm */
  (unsigned)(0) timeWhenSeen,         /**< Time stamp, indicating what its name says */
});

inline FilteredBallPercept::FilteredBallPercept(const Vector2f& posImg, const Vector2f& posFld,
                                                const Matrix2f& covFld, float r, unsigned t):
positionInImage(posImg), positionOnField(posFld),
covOnField(covFld), radiusOnField(r), timeWhenSeen(t) {}

/**
 * @struct FilteredBallPercepts
 *
 * A representation that holds a list (!) of recently perceived balls.
 */
STREAMABLE(FilteredBallPercepts,
{
  /** Draws the estimate on the field */
  void draw() const;

  /** Verifies that the filtered ball percepts contain valid values. */
  void verify() const;
  ,
  (std::vector<FilteredBallPercept, Eigen::aligned_allocator<FilteredBallPercept>>) percepts, /**< List of ball perceptions. Empty, if no new ball was seen. Newest ball is at position 0, older balls follow */
});
