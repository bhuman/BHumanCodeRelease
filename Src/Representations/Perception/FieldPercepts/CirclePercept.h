/**
 * @file CirclePercept.h
 * Declaration of a struct that represents the center circle.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct CirclePercept
 * A struct that represents the found field center circle.
 */
STREAMABLE(CirclePercept,
{
  /** The method draws the percepts to image/field/3D scene. */
  void draw() const;

  /** Verifies that this percept contains valid values. */
  void verify() const,

  (Vector2f)(Vector2f::Zero()) pos, /**< The position of the center of the center circle in field coordinates */
  (Matrix2f)((Matrix2f() << 1.f, 0.f, 0.f, 1.f).finished()) cov,  /**< The covariance of the center circle "measurement" */
  (bool)(false) wasSeen,            /**< Has the percept been seen in the last frame? */
});
