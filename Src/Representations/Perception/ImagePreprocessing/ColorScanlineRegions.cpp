#include "ColorScanlineRegions.h"
#include "Tools/Debugging/DebugDrawings.h"

void ColorScanlineRegionsVertical::draw() const
{
  static const ColorRGBA colors[] =
  {
    ColorRGBA::gray,
    ColorRGBA::white,
    ColorRGBA::black,
    ColorRGBA::green
  };

  DEBUG_DRAWING("representation:ColorScanlineRegionsVertical:image", "drawingOnImage")
  {
    for(const Scanline& line : scanlines)
    {
      for(const ScanlineRegion& r : line.regions)
      {
        LINE("representation:ColorScanlineRegionsVertical:image", line.x, r.range.from, line.x, r.range.to - 1, 4, Drawings::solidPen, colors[r.color]);
      }
    }
  }
}

void ColorScanlineRegionsVerticalClipped::draw() const
{
  static const ColorRGBA colors[] =
  {
    ColorRGBA::gray,
    ColorRGBA::white,
    ColorRGBA::black,
    ColorRGBA::green
  };

  DEBUG_DRAWING("representation:ColorScanlineRegionsVerticalClipped:image", "drawingOnImage")
  {
    for(const Scanline& line : scanlines)
    {
      for(const ScanlineRegion& r : line.regions)
      {
        LINE("representation:ColorScanlineRegionsVerticalClipped:image", line.x, r.range.from, line.x, r.range.to - 1, 4, Drawings::solidPen, colors[r.color]);
      }
    }
  }

  DEBUG_DRAWING("representation:ColorScanlineRegionsVerticalClipped:lowRes", "drawingOnImage")
  {
    for(size_t i = lowResStart; i < scanlines.size(); i += lowResStep)
    {
      const Scanline& line = scanlines[i];
      for(const ScanlineRegion& r : line.regions)
      {
        LINE("representation:ColorScanlineRegionsVerticalClipped:lowRes", line.x, r.range.from, line.x, r.range.to - 1, 4, Drawings::solidPen, colors[r.color]);
      }
    }
  }
}

void CompressedColorScanlineRegionsVertical::draw() const
{
  static const ColorRGBA colors[] =
  {
    ColorRGBA::gray,
    ColorRGBA::white,
    ColorRGBA::black,
    ColorRGBA::green
  };

  DEBUG_DRAWING("representation:CompressedColorScanlineRegionsVertical:image", "drawingOnImage")
  {
    for(const Scanline& line : scanlines)
    {
      for(const ScanlineRegion& r : line.regions)
      {
        LINE("representation:CompressedColorScanlineRegionsVertical:image", line.x, r.range.from, line.x, r.range.to - 1, 4, Drawings::solidPen, colors[r.color]);
      }
    }
  }
}

void ColorScanlineRegionsHorizontal::draw() const
{
  static const ColorRGBA colors[] =
  {
    ColorRGBA::gray,
    ColorRGBA::white,
    ColorRGBA::black,
    ColorRGBA::green
  };

  DEBUG_DRAWING("representation:ColorScanlineRegionsHorizontal:image", "drawingOnImage")
  {
    for(const Scanline& line : scanlines)
    {
      for(const ScanlineRegion& r : line.regions)
      {
        LINE("representation:ColorScanlineRegionsHorizontal:image", r.range.from, line.y, r.range.to - 1, line.y, 4, Drawings::solidPen, colors[r.color]);
      }
    }
  }
}
