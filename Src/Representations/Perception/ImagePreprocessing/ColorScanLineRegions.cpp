#include "ColorScanLineRegions.h"
#include "Debugging/DebugDrawings.h"

void ColorScanLineRegionsVerticalClipped::draw() const
{
  DEBUG_DRAWING("representation:ColorScanLineRegionsVerticalClipped:image", "drawingOnImage")
  {
    for(const ScanLine& line : scanLines)
    {
      for(const ScanLineRegion& r : line.regions)
      {
        LINE("representation:ColorScanLineRegionsVerticalClipped:image", line.x - 1, r.range.from, line.x - 1, r.range.to - 1, 1, Drawings::solidPen, r.getDrawColor());
        LINE("representation:ColorScanLineRegionsVerticalClipped:image", line.x, r.range.from, line.x, r.range.to - 1, 1, Drawings::solidPen, r.getDrawColor());
        LINE("representation:ColorScanLineRegionsVerticalClipped:image", line.x + 1, r.range.from, line.x + 1, r.range.to - 1, 1, Drawings::solidPen, r.getDrawColor());
      }
    }
  }

  DEBUG_DRAWING("representation:ColorScanLineRegionsVerticalClipped:lowRes", "drawingOnImage")
  {
    for(size_t i = lowResStart; i < scanLines.size(); i += lowResStep)
    {
      const ScanLine& line = scanLines[i];
      for(const ScanLineRegion& r : line.regions)
      {
        LINE("representation:ColorScanLineRegionsVerticalClipped:lowRes", line.x - 1, r.range.from, line.x - 1, r.range.to - 1, 1, Drawings::solidPen, r.getDrawColor());
        LINE("representation:ColorScanLineRegionsVerticalClipped:lowRes", line.x, r.range.from, line.x, r.range.to - 1, 1, Drawings::solidPen, r.getDrawColor());
        LINE("representation:ColorScanLineRegionsVerticalClipped:lowRes", line.x + 1, r.range.from, line.x + 1, r.range.to - 1, 1, Drawings::solidPen, r.getDrawColor());
      }
    }
  }
}

void ColorScanLineRegionsHorizontal::draw() const
{
  DEBUG_DRAWING("representation:ColorScanLineRegionsHorizontal:image", "drawingOnImage")
  {
    for(const ScanLine& line : scanLines)
    {
      for(const ScanLineRegion& r : line.regions)
      {
        LINE("representation:ColorScanLineRegionsHorizontal:image", r.range.from, line.y - 1, r.range.to - 1, line.y - 1, 1, Drawings::solidPen, r.getDrawColor());
        LINE("representation:ColorScanLineRegionsHorizontal:image", r.range.from, line.y, r.range.to - 1, line.y, 1, Drawings::solidPen, r.getDrawColor());
        LINE("representation:ColorScanLineRegionsHorizontal:image", r.range.from, line.y + 1, r.range.to - 1, line.y + 1, 1, Drawings::solidPen, r.getDrawColor());
      }
    }
  }
}
