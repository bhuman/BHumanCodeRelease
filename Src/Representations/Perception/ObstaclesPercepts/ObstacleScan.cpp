/**
 * @file ObstacleScan.cpp
 *
 * This file implements a representation of a collection of perceived obstacle points per image column.
 *
 * @author Arne Hasselbring
 */

#include "ObstacleScan.h"
#include "Debugging/DebugDrawings.h"
#include "Platform/BHAssert.h"

void ObstacleScan::draw() const
{
  DEBUG_DRAWING("representation:ObstacleScan:image", "drawingOnImage")
  {
    unsigned int x = xOffsetInImage;
    for(const int y : yLowerInImage)
    {
      if(y >= 0)
        LINE("representation:ObstacleScan:image", x - xOffsetInImage, y, x - xOffsetInImage + xStepInImage, y, 2, Drawings::dashedPen, ColorRGBA::violet);
      x += xStepInImage;
    }
  }

  DEBUG_DRAWING("representation:ObstacleScan:field", "drawingOnField")
  {
    ASSERT(pointsOnField.size() == yLowerInImage.size());
    bool hadPoint = false;
    for(std::size_t i = 0; i < pointsOnField.size() + 1; ++i)
    {
      const bool havePoint = i < pointsOnField.size() && yLowerInImage[i] >= 0;
      if(havePoint != hadPoint)
        CROSS("representation:ObstacleScan:field", pointsOnField[havePoint ? i : i - 1].x(), pointsOnField[havePoint ? i : i - 1].y(), 30, 10, Drawings::solidPen, ColorRGBA::violet);
      if(havePoint && hadPoint)
        LINE("representation:ObstacleScan:field", pointsOnField[i - 1].x(), pointsOnField[i - 1].y(), pointsOnField[i].x(), pointsOnField[i].y(), 10, Drawings::solidPen, ColorRGBA::violet);
      hadPoint = havePoint;
    }
  }
}
