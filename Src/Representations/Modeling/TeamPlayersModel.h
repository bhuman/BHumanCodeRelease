/**
 * @file TeamPlayersModel.h
 *
 * Declaration of a representation that represents a combined world model
 *
 * @author Katharina Gillmann
 * @pfusche Florian
 */

#pragma once

#include <vector>
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Modeling/ObstacleModel.h"

/**
 * @struct TeamPlayersModel
 * a combined obstacle model.
 */
STREAMABLE(TeamPlayersModel,
{
  void draw() const,

  (std::vector<Obstacle>) obstacles, /**< poses and covariance of own robots */
});
