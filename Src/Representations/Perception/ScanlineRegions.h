#pragma once

#include "Representations/Configuration/ColorTable.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ScanlineRegions,
{
  STREAMABLE(Region,
  {
    Region() = default;
    Region(const unsigned lower, const unsigned upper, const ColorTable::Colors c),

    (int) upper, // inclusive
    (int) lower, // exclusive
    (ColorTable::Colors) color,
  });

  STREAMABLE(Scanline,
  {
    Scanline() = default;
    Scanline(const unsigned x),

    (unsigned)(0) x,
    (std::vector<Region>) regions,
  });

private:
  DECLARE_DEBUG_IMAGE(RegionsReconstructed);

  bool overlap(const Region& r1, const Region& r2) const;
  void drawParallelogram(const Region& left, const Region& right,
                         const int leftX, const int rightX) const;
public:
  void draw() const,

  (std::vector<Scanline>) scanlines,
  (unsigned)(0) lowResStart,
  (unsigned)(1) lowResStep,
});

inline ScanlineRegions::Region::Region(const unsigned lower, const unsigned upper, const ColorTable::Colors c) :
  upper(upper), lower(lower), color(c)
{}

/**
 * A version of the ScanlineRegions that has been clipped at the FieldBoundary
 */
struct ScanlineRegionsClipped : public ScanlineRegions
{
  void draw() const;
};
