/**
 * @author Felix
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Configuration/FieldColors.h"

union ScanLineRange
{
  ScanLineRange() = default;
  ScanLineRange(const unsigned short from, const unsigned short to) : from(from), to(to) {}

  struct
  {
    unsigned short from; // upper/left; inclusive
    unsigned short to; // lower/right; exclusive
  };
  struct
  {
    unsigned short upper; // inclusive
    unsigned short lower; // exclusive
  };
  struct
  {
    unsigned short left; // inclusive
    unsigned short right; //  exclusive
  };
};

struct ScanLineRegion : public Streamable
{
  ScanLineRegion() = default;
  ScanLineRegion(const unsigned short from, const unsigned short to, const FieldColors::Color c);
  bool is(FieldColors::Color cmpColor) const { return cmpColor == color; }
  static const char* getName(FieldColors::Color e) { return TypeRegistry::getEnumName(e); }

  ScanLineRange range;
  FieldColors::Color color;

  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(ScanLineRegion);
    REG(range.from);
    REG(range.to);
    REG(color);
  }

protected:
  void serialize(In* in, Out* out);
};

inline ScanLineRegion::ScanLineRegion(const unsigned short from, const unsigned short to, const FieldColors::Color c) :
  range(from, to), color(c)
{}

inline void ScanLineRegion::serialize(In* in, Out* out)
{
  STREAM(range.from);
  STREAM(range.to);
  STREAM(color);
}

STREAMABLE(ColorScanLineRegionsVertical,
{
  STREAMABLE(ScanLine,
  {
    ScanLine() = default;
    ScanLine(const unsigned short x);
    ,
    (unsigned short)(0) x,
    (std::vector<ScanLineRegion>) regions,
  });

  void draw() const,

  (std::vector<ScanLine>) scanLines,
  (unsigned)(0) lowResStart, /**< First index of low res scanLines. */
  (unsigned)(1) lowResStep,  /**< Steps between low res scanLines. */
});

inline ColorScanLineRegionsVertical::ScanLine::ScanLine(const unsigned short x) :
  x(x), regions()
{
  regions.reserve(100);
}

/**
 * A version of the ColorScanLineRegionsVertical that has been clipped at the FieldBoundary
 */
STREAMABLE_WITH_BASE(ColorScanLineRegionsVerticalClipped, ColorScanLineRegionsVertical,
{
  void draw() const,
});

/**
 * A version of the ColorScanLineRegionsVertical which is scaled (linear interpolated?)
 */
STREAMABLE_WITH_BASE(CompressedColorScanLineRegionsVertical, ColorScanLineRegionsVertical,
{
  void draw() const,
});

STREAMABLE(ColorScanLineRegionsHorizontal,
{
  STREAMABLE(ScanLine,
  {
    ScanLine() = default;
    ScanLine(const unsigned short y);
    ,
    (unsigned short)(0) y,
    (std::vector<ScanLineRegion>) regions,
  });

  void draw() const,

  (std::vector<ScanLine>) scanLines,
});

inline ColorScanLineRegionsHorizontal::ScanLine::ScanLine(const unsigned short y) :
  y(y), regions()
{
  regions.reserve(100);
}
