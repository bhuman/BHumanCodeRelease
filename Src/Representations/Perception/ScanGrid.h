/**
 * The file declares a representation that describes the image grid that should be scanned.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ScanGrid,
{
  STREAMABLE(Line,
  {
    Line() = default;
    Line(int x, int y, unsigned yMaxIndex),

    (int) x, /**< x coordinate of the scanline. */
    (int) yMax, /**< Maximum y coordinate (exclusive). */
    (unsigned) yMaxIndex, /**< Index of the lowest y coordinate relevant for this scanline. */
  });

  void draw() const,

  (std::vector<int>) y, /**< All possible y coordinates of pixels to be scanned. */
  (std::vector<Line>) lines, /**< Decription of all scanlines. */
  (int)(0) fieldLimit, /**< Upper bound for all scanlines (exclusive). */
  (unsigned)(0) lowResStart, /**< First index of low res grid. */
  (unsigned)(1) lowResStep, /**< Steps between low res grid lines. */
});

inline ScanGrid::Line::Line(int x, int yMax, unsigned yMaxIndex) :
  x(x), yMax(yMax), yMaxIndex(yMaxIndex)
{}
