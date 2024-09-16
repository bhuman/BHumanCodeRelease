/**
 * The file declares a representation that describes the image grid that should be scanned.
 * @author Thomas Röfer
 * @author Lukas Malte Monnerjahn
 */

#include "ScanGrid.h"
#include "Debugging/DebugDrawings.h"

void ScanGrid::clear()
{
  fullResY.clear();
  lowResHorizontalLines.clear();
  verticalLines.clear();
}

bool ScanGrid::isValid() const
{
  return !fullResY.empty() && !lowResHorizontalLines.empty() && !verticalLines.empty() && fieldLimit >= 0;
}

void ScanGrid::draw() const
{
  DEBUG_DRAWING("representation:ScanGrid", "drawingOnImage")
  {
    for(const Line& verticalLine : verticalLines)
    {
      for(auto it = lowResHorizontalLines.cbegin() + verticalLine.lowResYMaxIndex; it != lowResHorizontalLines.cend(); ++it)
      {
        if(verticalLine.x >= it->left && verticalLine.x < it->right)
          LINE("representation:ScanGrid", verticalLine.x, it->y - 1, verticalLine.x, it->y, 2, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}
