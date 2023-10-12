/**
 * @file Keypoints.cpp
 *
 * This file implements a representation that represents the output of the
 * keypoint detector, i.e. pixel positions and confidences of 17 different
 * body parts.
 *
 * @author Thomas Röfer
 */

#include "Keypoints.h"
#include "Debugging/DebugDrawings.h"

void Keypoints::draw() const
{
  DEBUG_DRAWING("representation:Keypoints", "drawingOnImage")
  {
    FOREACH_ENUM(Keypoint, i)
    {
      const Point& point = points[i];
      if(point.valid)
      {
        MID_DOT("representation:Keypoints", point.position.x(), point.position.y(), ColorRGBA::red, ColorRGBA::red);
        TIP("representation:Keypoints", point.position.x(), point.position.y(), 5, TypeRegistry::getEnumName(i));
      }
    }
  }
  DEBUG_DRAWING("representation:Keypoints:patch", "drawingOnImage")
  {
    RECTANGLE("representation:Keypoints:patch", patchBoundary.x.min, patchBoundary.y.min,
              patchBoundary.x.max, patchBoundary.y.max, 1, Drawings::solidPen, ColorRGBA::red);
  }
}
