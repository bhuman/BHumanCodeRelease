/*
 * File:   ObstacleClusters.h
 * Author: Arne Böckmann arneboe@tzi.de
 */

#pragma once

#include "Tools/Math/Random.h"
#include "Representations/Modeling/CombinedWorldModel.h" //for GaussianPositionDistribution

STREAMABLE(ObstacleClusters,
{
public:
  /** Function for drawing */
  void draw() const,

  (std::vector<GaussianPositionDistribution>) obstacles, /**< Obstacles in absolute world coordinates */
});

STREAMABLE(ObstacleClustersCompressed,
{
public:
  STREAMABLE(Obstacle,
  {
  public:
    Obstacle() = default;
    Obstacle(const GaussianPositionDistribution& g);
    operator GaussianPositionDistribution() const,

    (Vector2<short>) position,
    (float) c00,
    (float) c01,
    (float) c11,
  });

  ObstacleClustersCompressed() = default;
  ObstacleClustersCompressed(const ObstacleClusters& obstacles, unsigned int maxNumOfObstaclesToSend);
  operator ObstacleClusters() const,

  (std::vector<Obstacle>) obstacles,
});
