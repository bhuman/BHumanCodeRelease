#include "ScanlineRegionClipper.h"
#include "Tools/Math/Geometry.h"
#include <iostream>
#include <algorithm>

MAKE_MODULE(ScanlineRegionClipper, Perception)

void ScanlineRegionClipper::update(ScanlineRegionsClipped& clippedRegions)
{
  clippedRegions.scanlines.resize(theScanlineRegions.scanlines.size());
  auto original = theScanlineRegions.scanlines.begin();
  auto clipped = clippedRegions.scanlines.begin();
  for(; original != theScanlineRegions.scanlines.end(); ++original, ++clipped)
  {
    clipped->regions.clear();
    clipped->x = original->x;
    int yBoundary = std::max(0, theFieldBoundary.getBoundaryY(original->x));
    yBoundary = std::min(yBoundary, theImage.height - 1);
    for(const ScanlineRegions::Region& region : original->regions)
    {
      ASSERT(region.lower >= 0 && region.lower <= theImage.height);
      ASSERT(region.upper >= 0 && region.upper < theImage.height);
      ASSERT(region.lower - region.upper > 0);
      if(region.lower <= yBoundary)//lower is exclusive, if it is equal to yBoundary the last pixel of the region will be above the boundary
      {//all following regions including this one will be above the boundary, skip them
        break;
      }
      else if(region.lower > yBoundary && region.upper <= yBoundary)
      {//only part of the region is above the boundary, clip it
        clipped->regions.emplace_back(region.lower, yBoundary, region.color);
        break;//following regions will be above the boundary, skip them
      }
      else
      {
        clipped->regions.emplace_back(region);
      }
    }
  }
}