/**
 * The file declares a module that creates colored regions on a number of vertical scanlines.
 * The scanlines provided here are sparse and meant for calculating the field boundary.
 * The upper bound of all scanlines is the upper image border and the back-projection of the
 * farthest point that could appear on the field back to the image. The lower bound is the
 * lower image border and the body contour.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "LineScanner.h"

MODULE(LowResScanlineRegionizer,
{,
  REQUIRES(ColorTable),
  REQUIRES(Image),
  REQUIRES(ScanGrid),
  PROVIDES(ScanlineRegions),
  DEFINES_PARAMETERS(
  {,
    (float)(0.5f) minColorRatio, /**< The ratio of pixels of a different color that is expected after an edge (relative to the step width). */
  }),
});

class LowResScanlineRegionizer : public LowResScanlineRegionizerBase
{
private:
  const LineScanner lineScanner;

  void update(ScanlineRegions& scanlineRegions);

public:
  LowResScanlineRegionizer() : lineScanner(theColorTable, theImage, theScanGrid) {}
};
