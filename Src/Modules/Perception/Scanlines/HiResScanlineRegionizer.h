/**
 * The file declares a module that creates colored regions on a number of vertical scanlines.
 * The module clips the colored regions of a low res grid to the field boundary and adds 
 * additional hi res regions in between. The upper bound of all scanlines is the field
 * boandary. The lower bound is the lower image border and the body contour.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/FieldBoundary.h"
#include "LineScanner.h"

MODULE(HiResScanlineRegionizer,
{,
  REQUIRES(ColorTable),
  REQUIRES(FieldBoundary),
  REQUIRES(Image),
  REQUIRES(ScanGrid),
  REQUIRES(ScanlineRegions),
  PROVIDES(ScanlineRegionsClipped),
  DEFINES_PARAMETERS(
  {,
    (float)(0.5f) minColorRatio, /**< The ratio of pixels of a different color that is expected after an edge (relative to the step width). */
  }),
});

class HiResScanlineRegionizer : public HiResScanlineRegionizerBase
{
private:
  const LineScanner lineScanner;

  void update(ScanlineRegionsClipped& scanlineRegions);

public:
  HiResScanlineRegionizer() : lineScanner(theColorTable, theImage, theScanGrid) {}
};
