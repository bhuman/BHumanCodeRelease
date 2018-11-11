/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanlineRegions.h"

#include <vector>

MODULE(ColorScanlineRegionizer,
{,
  REQUIRES(ECImage),
  REQUIRES(ScanGrid),
  PROVIDES(ColorScanlineRegionsVertical),
  PROVIDES(ColorScanlineRegionsHorizontal),
  REQUIRES(ColorScanlineRegionsVertical),
  DEFINES_PARAMETERS(
  {,
    (float)(0.5f) minColorRatio, /**< The ratio of pixels of a different color that is expected after an edge (relative to the step width). */
    (unsigned short)(8) minHorizontalScanlineDistance,
    (unsigned short)(4) minHorizontalRegionSize,
  }),
});

class ColorScanlineRegionizer : public ColorScanlineRegionizerBase
{
private:
  void update(ColorScanlineRegionsVertical& colorScanlineRegionsVertical) override;
  void update(ColorScanlineRegionsHorizontal& colorScanlineRegionsHorizontal) override;

  void scanVertical(const ScanGrid::Line& line, const int top, std::vector<ScanlineRegion>& regions) const;
};
