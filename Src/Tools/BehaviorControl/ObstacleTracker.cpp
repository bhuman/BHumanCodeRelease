/**
 * @file ObstacleTracker.cpp
 *
 * This file implements a utility class to track a specific obstacle with certain properties.
 *
 * @author Arne Hasselbring
 */

#include "ObstacleTracker.h"
#include "Platform/BHAssert.h"

ObstacleTracker::ObstacleTracker(const std::function<bool(Vector2f&)>& selectNewObstacle, const std::function<bool(Vector2f&)>& trackObstacle) :
  selectNewObstacle(selectNewObstacle),
  trackObstacle(trackObstacle)
{}

void ObstacleTracker::updateTrackedObstacle()
{
  if(trackedObstacleHasBeenCalculated)
    return;
  trackedObstacleHasBeenCalculated = true;

  if(trackedObstacleExists && (trackedObstacleExists = trackObstacle(trackedObstacleOnField)))
    return;

  ASSERT(!trackedObstacleExists);
  trackedObstacleExists = selectNewObstacle(trackedObstacleOnField);
}

bool ObstacleTracker::hasTrackedObstacle() const
{
  const_cast<ObstacleTracker*>(this)->updateTrackedObstacle();
  return trackedObstacleExists;
}

Vector2f ObstacleTracker::getTrackedObstacleOnField() const
{
  ASSERT(trackedObstacleHasBeenCalculated);
  ASSERT(trackedObstacleExists);
  return trackedObstacleOnField;
}

void ObstacleTracker::preProcess()
{
  trackedObstacleHasBeenCalculated = false;
}

void ObstacleTracker::postProcess()
{
  if(!trackedObstacleHasBeenCalculated)
    trackedObstacleExists = false;
}
