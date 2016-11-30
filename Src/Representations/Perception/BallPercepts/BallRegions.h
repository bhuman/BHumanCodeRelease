/**
 * @file BallRegions.h
 * Declaration of a struct that represents the regions around ball spots that must be scanned
 * for a ball detection.
 * @author Thomas Röfer
 *
 */

#pragma once

#include "Tools/Boundary.h"
#include "Tools/Debugging/DebugDrawings.h"

/** The regions that must be searched for the center of the ball. */
STREAMABLE(BallRegions,
{
  BallRegions()
  {
    regions.reserve(50);
  }

  void draw() const
  {
    DEBUG_DRAWING("representation:BallRegions", "drawingOnImage")
      for(const Boundaryi& region : regions)
        RECTANGLE("representation:BallRegions", region.x.min, region.y.min,
                  region.x.max, region.y.max, 1, Drawings::solidPen, ColorRGBA::orange);
  },

  (std::vector<Boundaryi>) regions,
});

/** The regions that the CNS must be calculated of to search for the ball. */
struct CNSRegions : public BallRegions
{
  void draw() const
  {
    DEBUG_DRAWING("representation:CNSRegions", "drawingOnImage")
     for(const Boundaryi& region : regions)
       RECTANGLE("representation:CNSRegions", region.x.min, region.y.min,
                 region.x.max, region.y.max, 1, Drawings::solidPen, ColorRGBA::red);
  }
};
