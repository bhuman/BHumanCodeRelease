/*
 * File:   ObstacleWheel.h
 * Author: arne
 *
 * Created on March 4, 2013, 4:23 PM
 */

#pragma once

#include <vector>
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ObstacleWheel,
{
public:
  STREAMABLE(Spot,
  {,
    (Vector2<>) position, /**< The obstacle that was last seen */
    (int)(0) seenCount, /**< Number of continious frames in which this spot has been seen */
    (CameraInfo, Camera)(upper) seenBy, /**< which camera has seen the spot */
  });

  STREAMABLE(Cone,
  {,
    (float)(0) angle, /**< angle of the center of this cone in rad */
    (float)(0) distance, /**< Distance to the obstacle */
    (float)(0) angleDistance, /**< Distance to the obstacle expressed as angle */
    (bool)(false) hasObstacle, /**< Whether this cone contains an obstacle or not */
    (Spot) spot,
    (bool)(false) seenThisFrame, /**< Whether the obstacle in this cone has been seen this frame */
  });

private:
  void drawCone(const Cone& cone) const;

public:
  void draw() const,

  (float) coneWidth, /**< width of one cone in rad */
  (int) wheelRadius, /**< in mm */

  /** Sensor cones. Left most cone is from 0 rad to coneWidth.
   *  0 rad is directly in front of the robot */
  (std::vector<Cone>) cones,
});
