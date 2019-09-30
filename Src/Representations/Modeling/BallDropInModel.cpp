/**
 * @file BallDropInModel.cpp
 *
 * This file implements the draw method for the BallDropInModel representation.
 *
 * @author Arne Hasselbring, Nicole Schrader
 */

#include "BallDropInModel.h"
#include "Tools/Debugging/DebugDrawings.h"

void BallDropInModel::draw() const
{
  DEBUG_DRAWING("representation:BallDropInModel:outPosition", "drawingOnField")
  {
    CROSS("representation:BallDropInModel:outPosition", outPosition.x(), outPosition.y(),
          75, 30, Drawings::solidPen, ColorRGBA(255, 192, 203));
  }

  DEBUG_DRAWING("representation:BallDropInModel:dropInPositions", "drawingOnField")
  {
    for(Vector2f position : dropInPositions)
    {
      CROSS("representation:BallDropInModel:dropInPositions", position.x(), position.y(),
            75, 30, Drawings::solidPen, ColorRGBA(255, 192, 203));
    }
  }
}
