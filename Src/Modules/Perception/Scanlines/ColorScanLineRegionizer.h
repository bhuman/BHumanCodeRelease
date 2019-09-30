/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"

#include <vector>

MODULE(ColorScanLineRegionizer,
{,
  REQUIRES(ECImage),
  REQUIRES(ScanGrid),
  PROVIDES(ColorScanLineRegionsVertical),
  PROVIDES(ColorScanLineRegionsHorizontal),
  REQUIRES(ColorScanLineRegionsVertical),
  DEFINES_PARAMETERS(
  {,
    (float)(0.5f) minColorRatio, /**< The ratio of pixels of a different color that is expected after an edge (relative to the step width). */
    (unsigned short)(8) minHorizontalScanLineDistance,
    (unsigned short)(4) minHorizontalRegionSize,
  }),
});

class ColorScanLineRegionizer : public ColorScanLineRegionizerBase
{
private:
  void update(ColorScanLineRegionsVertical& colorScanLineRegionsVertical) override;
  void update(ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal) override;

  void scanVertical(const ScanGrid::Line& line, const int top, std::vector<ScanLineRegion>& regions) const;
};
