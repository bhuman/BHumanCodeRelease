/**
 * @file ObstacleModel.h
 *
 * Declaration of struct ObstacleModel.
 *
 * @author Florian Maaß
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Modeling/Obstacle.h"

/**
 * @struct ObstacleModel
 *
 * A struct that represents all kind of obstacles seen by vision, detected by arm contact,
 * foot bumper contact.
 */

STREAMABLE(ObstacleModel,
{
  ObstacleModel() = default;
  void draw() const,

  (std::vector<Obstacle>) obstacles, /**< List of obstacles (all entries are somewhat valid obstacles)*/
});

STREAMABLE(ObstacleModelCompressed,
{
  // Definition of an compressed obstacle
  STREAMABLE(ObstacleCompressed,
  {
    ObstacleCompressed() = default;
    ObstacleCompressed(const Obstacle& other),

    (float) covXX,
    (float) covYY,
    (float) covXY,                        /**< Covariance matrix of an obstacle */
    (Vector2f) center,                    /**< Center point of an Obstacle */
    (Vector2f) left,                      /**< Left point of an Obstacle */
    (Vector2f) right,                     /**< Right point of an Obstacle */
    ((Obstacle) Type) type, /**< See enumeration 'Type' above */
  });
  ObstacleModelCompressed() = default;
  ObstacleModelCompressed(const ObstacleModel& other, size_t maxNumberOfObstacles);
  void draw() const,

  (std::vector<ObstacleCompressed>) obstacles, /**< List of obstacles (all entries are somewhat valid obstacles)*/
});
