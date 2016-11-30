/**
 * @file ObstacleModel.h
 *
 * Declaration of struct ObstacleModel.
 *
 * @author Florian Maaß
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
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
  void draw() const;
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< List of obstacles (position relative to own pose) */
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
    (Vector2s) center,                    /**< Center point of an obstacle */
    (Vector2s) left,                      /**< Left point of an obstacle */
    (Vector2s) right,                     /**< Right point of an obstacle */
    (unsigned int) lastSeen,              /**< Timestamp of last measurement */
    ((Obstacle) Type) type,               /**< See enumeration 'Type' in Obstacle.h */
  });
  ObstacleModelCompressed() = default;
  ObstacleModelCompressed(const ObstacleModel& other, size_t maxNumberOfObstacles);
  void draw() const,

  (std::vector<ObstacleCompressed>) obstacles, /**< List of obstacles */
});
