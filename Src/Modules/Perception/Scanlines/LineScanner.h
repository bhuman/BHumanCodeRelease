/**
 * The file declares a class that creates colored regions on a single vertical scanline.
 * The class is based on Arne Böckmann's original implementation of this idea.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Perception/ScanGrid.h"
#include "Representations/Perception/ScanlineRegions.h"

class LineScanner
{
private:
  const ColorTable& colorTable;
  const Image& image;
  const ScanGrid& scanGrid;

public:
  LineScanner(const ColorTable& colorTable,
              const Image& image,
              const ScanGrid& scanGrid)
  : colorTable(colorTable),
    image(image),
    scanGrid(scanGrid) {}

  /**
   * Scan vertically and add a new scanline with colored regions.
   * @param line Description of the line to be scanned.
   * @param top Upper bound for the pixels scanned (exclusive).
   * @param minColorRatio The ratio of pixels of a different color that is expected 
   *                      after an edge (relative to the step width).
   * @param scanlineRegions A new scanline containing the regions found will be added
   *                        to this object.
   */
  void scan(const ScanGrid::Line& line, const int top, const float minColorRatio, ScanlineRegions& scanlineRegions) const;
};
