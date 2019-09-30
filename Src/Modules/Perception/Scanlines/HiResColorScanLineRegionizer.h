/**
 * @author Felix Thielke
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ScanGrid.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanLineRegions.h"

#include <vector>

MODULE(HiResColorScanLineRegionizer,
{,
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(ColorScanLineRegionsVertical),
  REQUIRES(ScanGrid),

  PROVIDES(ColorScanLineRegionsVerticalClipped),
  DEFINES_PARAMETERS(
  {,
    (float)(0.5f) minColorRatio, /**< The ratio of pixels of a different color that is expected after an edge (relative to the step width). */
    (unsigned short)(8) minHorizontalScanLineDistance,
  }),
});

class HiResColorScanLineRegionizer : public HiResColorScanLineRegionizerBase
{
private:
  void update(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped) override;

  void scanVertical(const ScanGrid::Line& line, const int top, std::vector<ScanLineRegion>& regions) const;
};
