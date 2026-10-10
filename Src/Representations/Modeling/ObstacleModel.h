/**
 * @file ObstacleModel.h
 *
 * Declaration of a representation that stores a list of obstacles.
 *
 * @author Florian Maaß
 */
#pragma once

#include "Math/Eigen.h"
#include "Tools/Modeling/Obstacle.h"

/**
 * @struct ObstacleModel
 *
 * A struct that represents all types of obstacles.
 */
STREAMABLE(ObstacleModel,
{
  /** Draw the content of this representation */
  void draw() const;

  /** Verify the data consistency */
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< List of obstacles (in robot-relative field coordinates) */
});
