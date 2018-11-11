/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanlineRegions.h"

#include <vector>

MODULE(HiResColorScanlineRegionizer,
{,
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(ColorScanlineRegionsVertical),
  REQUIRES(ScanGrid),

  PROVIDES(ColorScanlineRegionsVerticalClipped),
  DEFINES_PARAMETERS(
  {,
    (float)(0.5f) minColorRatio, /**< The ratio of pixels of a different color that is expected after an edge (relative to the step width). */
    (unsigned short)(8) minHorizontalScanlineDistance,
  }),
});

class HiResColorScanlineRegionizer : public HiResColorScanlineRegionizerBase
{
private:
  void update(ColorScanlineRegionsVerticalClipped& colorScanlineRegionsVerticalClipped) override;

  void scanVertical(const ScanGrid::Line& line, const int top, std::vector<ScanlineRegion>& regions) const;
};
