/**
 * @file PenaltyMarkSpots.h
 * Declaration of a struct that represents a spots that might be an indication of a PenaltyMark.
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Debugging/DebugDrawings.h"

/**
 * @struct PenaltyMarkSpots
 * A struct that represents spots that might be an indication of a penalty mark.
 */
STREAMABLE(PenaltyMarkSpots,
{
  PenaltyMarkSpots()
  {
    spots.reserve(20);
  }

  /** The method draws all penalty mark spots. */
  void draw() const
  {
    DEBUG_DRAWING("representation:PenaltyMarkSpots", "drawingOnImage") // Draws the penaltyMarkSpots to the image
    {
      for(const Vector2i& spot : spots)
      {
        CROSS("representation:PenaltyMarkSpots", spot.x(), spot.y(), 2, 3, Drawings::solidPen, ColorRGBA::violet);
        CROSS("representation:PenaltyMarkSpots", spot.x(), spot.y(), 2, 0, Drawings::solidPen, ColorRGBA::gray);
      }
    }
  },

  (std::vector<Vector2i>) spots,
});
