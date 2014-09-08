#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Infrastructure/Image.h"

/**
 * Removes regions outside the field boundary from the ScanlineRegions
 */
MODULE(ScanlineRegionClipper,
{,
  REQUIRES(ScanlineRegions),
  REQUIRES(FieldBoundary),
  REQUIRES(Image), //only for asserts
  PROVIDES_WITH_DRAW(ScanlineRegionsClipped),
});

class ScanlineRegionClipper : public ScanlineRegionClipperBase
{
private:
  void update(ScanlineRegionsClipped& clippedRegions);
};
