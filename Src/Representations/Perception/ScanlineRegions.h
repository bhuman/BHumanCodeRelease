#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Configuration/ColorTable.h"
#include <vector>
#include "Tools/Debugging/DebugImages.h"

STREAMABLE(ScanlineRegions,
{
public:
  using Color = ColorTable::Colors;
  STREAMABLE(Region,
  {
  public:
    Region() = default;
    Region(const unsigned lower, const unsigned upper, const Color& c);
    Region(const Region& other),

    (int) upper, // inclusive
    (int) lower, // exclusive
    (Color) color,
  });
  using RegionIterator = std::vector<Region>::const_iterator;


  STREAMABLE(Scanline,
  {
  public:
    Scanline() = default;
    Scanline(const unsigned x),

    (unsigned)(0) x,
    (std::vector<Region>) regions,
  });
  DECLARE_DEBUG_IMAGE(RegionsReconstructed);
  bool overlap(const Region& r1, const Region& r2) const;
  void drawParallelogram(const Region& left, const Region& right,
                         const int leftX, const int rightX) const;
  void draw(),

  (std::vector<Scanline>) scanlines,
});

/**
 * A version of the ScanlineRegions that has been clipped at the FieldBoundary
 */
class ScanlineRegionsClipped : public ScanlineRegions
{
public:
  void draw();
};