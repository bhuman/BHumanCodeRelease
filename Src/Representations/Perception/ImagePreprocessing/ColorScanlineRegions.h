/**
 * @author Felix
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Configuration/FieldColors.h"

union ScanlineRange
{
  ScanlineRange() = default;
  ScanlineRange(const unsigned short from, const unsigned short to) : from(from), to(to) {}

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

struct ScanlineRegion : public Streamable
{
  ScanlineRegion() = default;
  ScanlineRegion(const unsigned short from, const unsigned short to, const FieldColors::Color c);
  bool is(FieldColors::Color cmpColor) const { return cmpColor == color; }
  static const char* getName(FieldColors::Color e) { return FieldColors::getName(e); }

  ScanlineRange range;
  FieldColors::Color color;

private:
  void serialize(In* in, Out* out);
};

inline ScanlineRegion::ScanlineRegion(const unsigned short from, const unsigned short to, const FieldColors::Color c) :
  range(from, to), color(c)
{}

inline void ScanlineRegion::serialize(In * in, Out * out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(color);
  STREAM(range.from);
  STREAM(range.to);
  STREAM_REGISTER_FINISH;
}

STREAMABLE(ColorScanlineRegionsVertical,
{
  STREAMABLE(Scanline,
  {
    Scanline() = default;
    Scanline(const unsigned short x);
    ,
    (unsigned short)(0) x,
    (std::vector<ScanlineRegion>) regions,
  });

  void draw() const,

  (std::vector<Scanline>) scanlines,
  (unsigned)(0) lowResStart, /**< First index of low res scanlines. */
  (unsigned)(1) lowResStep,  /**< Steps between low res scanlines. */
});

inline ColorScanlineRegionsVertical::Scanline::Scanline(const unsigned short x) :
  x(x), regions()
{
  regions.reserve(100);
}


/**
* A version of the ColorScanlineRegionsVertical that has been clipped at the FieldBoundary
*/
struct ColorScanlineRegionsVerticalClipped : public ColorScanlineRegionsVertical
{
  void draw() const;
};

STREAMABLE(ColorScanlineRegionsHorizontal,
{
  STREAMABLE(Scanline,
  {
    Scanline() = default;
    Scanline(const unsigned short y);
    ,
    (unsigned short)(0) y,
    (std::vector<ScanlineRegion>) regions,
  });

  void draw() const,

  (std::vector<Scanline>) scanlines,
});

inline ColorScanlineRegionsHorizontal::Scanline::Scanline(const unsigned short y) :
  y(y), regions()
{
  regions.reserve(100);
}
