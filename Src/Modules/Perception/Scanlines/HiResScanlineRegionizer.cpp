/**
 * The file implements a module that creates colored regions on a number of vertical scanlines.
 * The module clips the colored regions of a low res grid to the field boundary and adds
 * additional hi res regions in between. The upper bound of all scanlines is the field
 * boandary. The lower bound is the lower image border and the body contour.
 * @author Thomas Röfer
 */

#include "HiResScanlineRegionizer.h"
#include <algorithm>

MAKE_MODULE(HiResScanlineRegionizer, perception)

void HiResScanlineRegionizer::update(ScanlineRegionsClipped& scanlineRegionsClipped)
{
  scanlineRegionsClipped.scanlines.clear();
  scanlineRegionsClipped.scanlines.reserve(theScanGrid.lines.size());
  scanlineRegionsClipped.lowResStart = theScanGrid.lowResStart;
  scanlineRegionsClipped.lowResStep = theScanGrid.lowResStep;

  if(theScanGrid.lines.empty())
    return;

  ASSERT(SystemCall::getMode() == SystemCall::logfileReplay ||
         (theScanGrid.lines.size() + theScanGrid.lowResStep - theScanGrid.lowResStart - 1) / theScanGrid.lowResStep == theScanlineRegions.scanlines.size());

  int i = 0;
  auto j = theScanlineRegions.scanlines.data();
  for(const ScanGrid::Line& line : theScanGrid.lines)
  {
    const int yBoundary = std::min(std::max(0, theFieldBoundary.getBoundaryY(line.x)), theImage.height - 1);
    if((i - static_cast<int>(theScanGrid.lowResStart)) % theScanGrid.lowResStep == 0)
    {
      scanlineRegionsClipped.scanlines.emplace_back(line.x);
      auto& src = (j++)->regions;
      auto& dest = scanlineRegionsClipped.scanlines.back().regions;
      auto k = src.begin();
      for(; k != src.end() && k->upper > yBoundary; ++k)
        dest.emplace_back(*k);
      if(k != src.end() && k->lower > yBoundary)
        dest.emplace_back(k->lower, yBoundary, k->color);
    }
    else
      lineScanner.scan(line, yBoundary - 1, minColorRatio, scanlineRegionsClipped);
    ++i;
  }
}
