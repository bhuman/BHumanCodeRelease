/*
 * File:   ObstacleSpots.h
 * Author: arne
 *
 * Created on February 7, 2013, 9:41 AM
 */


#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>
#include "Representations/Infrastructure/Image.h"
#include <cstring>

/**
 * Obstacle spots are spots at the junction of obstacles and the ground.
 * Usually at the feet of other robots.
 * Obstacle spots will also occur at the field border.
 */
STREAMABLE(ObstacleSpots,
{
public:
  typedef Vector2<int> Spot;

  /**An Obstacle consists of several spots clustered together*/
  STREAMABLE(Obstacle,
  {,
    (std::vector<Spot>) spots, /**< All spots that this obstacle is made up of. All of them are on the field plane */
    (Spot) centerOfMass, /**< The center of mass of all spots. Usually this is what you want to use when working with obstacles */

    /**A rectangle that shows how big a robot would be if it would be standing at the centerOfMass
       note: this rectangle can be bigger than the image*/
    (Vector2<int>) banZoneTopLeft,
    (Vector2<int>) banZoneBottomRight,
  });

  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:ObstacleSpots:spots", "drawingOnImage");
    DECLARE_DEBUG_DRAWING("representation:ObstacleSpots:banZones", "drawingOnImage");
    for(unsigned i = 0; i < obstacles.size(); ++i)
    {
      const Obstacle& c = obstacles[i];
      for(const Spot& s : c.spots)
      {
        CROSS("representation:ObstacleSpots:spots", s.x, s.y, 3, 3, Drawings::ps_solid, ColorClasses::white);
      }
      RECTANGLE("representation:ObstacleSpots:banZones", c.banZoneTopLeft.x, c.banZoneTopLeft.y,
                c.banZoneBottomRight.x, c.banZoneBottomRight.y, 6, Drawings::ps_solid, ColorClasses::white);
    }
  },

  (std::vector<Obstacle>) obstacles, /** Contains all obstacles */

  // Initialization
  obstacles.reserve(200);
});
