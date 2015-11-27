/**
 * The file declares a representation that describes the image grid that should be scanned.
 * @author Thomas Röfer
 */

#include "ScanGrid.h"
#include "Tools/Debugging/DebugDrawings.h"

void ScanGrid::draw() const
{
  DEBUG_DRAWING("representation:ScanGrid", "drawingOnImage")
  {
    for(const Line& line : lines)
    {
      for(auto i = y.begin() + line.yMaxIndex; i != y.end(); ++i)
      {
        LINE("representation:ScanGrid", line.x, *i - 1, line.x, *i, 2, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}
