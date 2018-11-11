/**
 * @file ImageRegions.h
 * Declaration of a struct that represents the regions around image spots that must be scanned
 * for a certain kind of percept detection.
 * @author Thomas Röfer
 *
 */

#pragma once

#include "Tools/Boundary.h"
#include "Tools/Debugging/DebugDrawings.h"

/** The regions around image spots that must be scanned for a certain kind of percept detection */
STREAMABLE(ImageRegions,
{
  ImageRegions()
  {
    regions.reserve(50);
  },

  (std::vector<Boundaryi>) regions,
});

/** The regions that must be searched for the center of the ball. */
STREAMABLE_WITH_BASE(BallRegions, ImageRegions,
{
  void draw() const
  {
    DEBUG_DRAWING("representation:BallRegions", "drawingOnImage")
      for(const Boundaryi& region : regions)
        RECTANGLE("representation:BallRegions", region.x.min, region.y.min,
                  region.x.max, region.y.max, 1, Drawings::solidPen, ColorRGBA::orange);
  },
});

/** The regions that the CNS must be calculated of to search for the ball. */
STREAMABLE_WITH_BASE(CNSRegions, ImageRegions,
{
  void draw() const
  {
    DEBUG_DRAWING("representation:CNSRegions", "drawingOnImage")
      for(const Boundaryi& region : regions)
        RECTANGLE("representation:CNSRegions", region.x.min, region.y.min,
                  region.x.max, region.y.max, 1, Drawings::solidPen, ColorRGBA::red);
  },
});

/** The regions that must be searched for the center of a penalty mark. */
STREAMABLE_WITH_BASE(PenaltyMarkRegions, ImageRegions,
{
  void draw() const
  {
    DEBUG_DRAWING("representation:PenaltyMarkRegions", "drawingOnImage")
      for(const Boundaryi& region : regions)
        RECTANGLE("representation:PenaltyMarkRegions", region.x.min, region.y.min,
                  region.x.max, region.y.max, 1, Drawings::solidPen, ColorRGBA::magenta);
  },
});

/** The regions that the CNS must be calculated of to search for a penalty mark. */
STREAMABLE_WITH_BASE(CNSPenaltyMarkRegions, ImageRegions,
{
  void draw() const
  {
    DEBUG_DRAWING("representation:CNSPenaltyMarkRegions", "drawingOnImage")
      for(const Boundaryi& region : regions)
        RECTANGLE("representation:CNSPenaltyMarkRegions", region.x.min, region.y.min,
                  region.x.max, region.y.max, 1, Drawings::solidPen, ColorRGBA::blue);
  },
});
