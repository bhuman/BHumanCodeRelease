/**
 * @file CirclePercept.h
 * Declaration of a struct that represents the center circle.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

/**
 * @struct CirclePercept
 * A struct that represents the found field center circle.
 */
STREAMABLE(CirclePercept,
{
  /**
   * The method draws the percepts to image/field/3D scene.
   */
  void draw() const,

  (Vector2f)(Vector2f::Zero()) pos, /**< The position of the center of the center circle in field coordinates */
  (bool)(false) wasSeen, /**< Has the percept been seen in the last frame? */
});
