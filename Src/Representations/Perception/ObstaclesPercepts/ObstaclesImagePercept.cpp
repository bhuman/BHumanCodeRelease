/**
 * @file ObstaclesImagePercept.cpp
 *
 * This file implements a representation that lists the obstacles that were detected in
 * the current image.
 *
 * @author Michel Bartsch
 * @author Andre Mühlenbrock
 * @author Thomas Röfer
 */

#include "ObstaclesImagePercept.h"
#include "Debugging/DebugDrawings.h"

void ObstaclesImagePercept::draw() const
{
  DEBUG_DRAWING("representation:ObstaclesImagePercept:image", "drawingOnImage")
  {
    for(const auto& obstacle : obstacles)
      RECTANGLE("representation:ObstaclesImagePercept:image", obstacle.left, obstacle.top, obstacle.right, obstacle.bottom, 4, Drawings::solidPen, ColorRGBA::white);
  }
}
