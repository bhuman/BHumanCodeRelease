/**
 * @file ObstaclesImagePercept.h
 *
 * This file declares a representation that lists the obstacles that were detected in
 * the current image.
 *
 * @author Michel Bartsch
 * @author Andre Mühlenbrock
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Eigen.h"

STREAMABLE(ObstaclesImagePercept,
{
  STREAMABLE(Obstacle,
  {,
    (int) top, /**< The guessed top border of the obstacle in the image. */
    (int) bottom, /**< The lower border of the obstacle in the image. */
    (int) left, /**< The left border of the obstacle in the image. This usually only includes the width at the lower end. */
    (int) right, /**< The right border of the obstacle in the image. This usually only includes the width at the lower end. */
    (bool) bottomFound, /**< Was the lower end of the obstacle found? Otherwise, it was hidden by the lower image border. */
    (bool) fallen, /**< Is the obstacle a player lying on the field? */
    (float)(1.f) confidence, /**< Confidence of the object prediction */
    (float)(-1.f) distance,
  });

  /** Draws this percept. */
  void draw() const,

  (std::vector<Obstacle>) obstacles, /**< All the obstacles found in the current image. */
});
