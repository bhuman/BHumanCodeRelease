/**
 * @file BallDropInModel.cpp
 *
 * This file implements the draw method for the BallDropInModel representation.
 *
 * @author Arne Hasselbring
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

  DEBUG_DRAWING("representation:BallDropInModel:dropInPosition", "drawingOnField")
  {
    CROSS("representation:BallDropInModel:dropInPosition", dropInPosition.x(), dropInPosition.y(),
          75, 30, Drawings::solidPen, ColorRGBA(255, 192, 203));
  }
}
