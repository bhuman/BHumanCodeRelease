/**
 * @file PenaltyMarkPercept.h
 * Declaration of a struct that represents a penalty mark.
 * @author Maik Schünemann
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct PenaltyMarkPercept
 * A struct that represents a detected penalty mark.
 */
STREAMABLE(PenaltyMarkPercept,
{
  /** Debug drawings for this percept */
  void draw() const;

  /** Verifies that this percept contains valid values. */
  void verify() const,

  (Vector2i)(Vector2i::Zero()) positionInImage, /**< Position in the image. */
  (Vector2f)(Vector2f::Zero()) positionOnField, /**< Position relative to robot on the field. */
  (Matrix2f)((Matrix2f() << 1.f, 0.f, 0.f, 1.f).finished()) covarianceOnField, /**< The measurement covariance of positionOnField */
  (bool)(false) wasSeen,                        /**< Indicates whether the feature has been seen in the last frame */
});
