/**
 * @file ObstacleTracker.h
 *
 * This file declares a utility class to track a specific obstacle with certain properties.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Eigen.h"
#include <functional>

class ObstacleTracker
{
public:
  /** Constructor. */
  ObstacleTracker(const std::function<bool(Vector2f&)>& selectNewObstacle, const std::function<bool(Vector2f&)>& trackObstacle);

  /** Updates the tracked obstacle by either tracking the old one or selecting a new one. */
  void updateTrackedObstacle();

  /**
   * Checks whether there is an obstacle that is being tracked.
   * @return Whether there is an obstacle that is being tracked.
   */
  bool hasTrackedObstacle() const;

  /**
   * Returns the tracked obstacle in field coordinates.
   * @return The tracked obstacle in field coordinates.
   */
  Vector2f getTrackedObstacleOnField() const;

  /** Must be called every frame before any other method of this class is called. */
  void preProcess();

  /** Must be called every frame after any other method of this class is called. */
  void postProcess();

private:
  std::function<bool(Vector2f&)> selectNewObstacle; /**< A function which decides whether a new tracked obstacle could be selected (and sets it). */
  std::function<bool(Vector2f&)> trackObstacle; /**< A function which decides whether the previous obstacle is still present (and updates it). */

  bool trackedObstacleHasBeenCalculated; /**< Whether the blocked obstacle has been updated this frame. */
  bool trackedObstacleExists = false; /**< Whether a blocked obstacle was found. */
  Vector2f trackedObstacleOnField; /**< The position of the blocked obstacle on the field. */
};
