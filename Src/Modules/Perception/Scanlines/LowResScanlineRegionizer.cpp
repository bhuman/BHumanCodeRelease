/**
 * The file implements a module that creates colored regions on a number of vertical scanlines.
 * The scanlines provided here are sparse and meant for calculating the field boundary.
 * The upper bound of all scanlines is the upper image border and the back-projection of the
 * farthest point that could appear on the field back to the image. The lower bound is the
 * lower image border and the body contour.
 * @author Thomas Röfer
 */

#include "LowResScanlineRegionizer.h"

MAKE_MODULE(LowResScanlineRegionizer, perception)

void LowResScanlineRegionizer::update(ScanlineRegions& scanlineRegions)
{
  scanlineRegions.scanlines.clear();
  scanlineRegions.lowResStart = 0;
  scanlineRegions.lowResStep = 1;

  if(theScanGrid.lines.empty())
    return;

  scanlineRegions.scanlines.reserve((theScanGrid.lines.size() + theScanGrid.lowResStep - theScanGrid.lowResStart - 1) / theScanGrid.lowResStep);
  for(size_t i = theScanGrid.lowResStart; i < theScanGrid.lines.size(); i += theScanGrid.lowResStep)
    lineScanner.scan(theScanGrid.lines[i], theScanGrid.fieldLimit, minColorRatio, scanlineRegions);
}
