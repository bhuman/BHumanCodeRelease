/**
 * The file declares a representation that describes the image grid that should be scanned.
 * @author Thomas Röfer
 * @author Lukas Malte Monnerjahn
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(ScanGrid,
{
  STREAMABLE(Line,
  {
    Line() = default;
    Line(int x, int yMin, int yMax, unsigned lowResYMaxIndex, unsigned yMaxIndex),

    (int) x, /**< x coordinate of the scanLine. */
    (int) yMin, /**< Minimum y coordinate (inclusive). */
    (int) yMax, /**< Maximum y coordinate (exclusive). */
    (unsigned) lowResYMaxIndex, /**< Index of the lowest low resolution horizontal scanLine relevant for this scanLine. */
    (unsigned) yMaxIndex, /**< Index of the lowest y coordinate relevant for this scanLine. */
  });

  STREAMABLE(HorizontalLine,
  {
    HorizontalLine() = default;
    HorizontalLine(int y, int left, int right),

    (int) y, /**< y coordinates of the scanLine. */
    (int) left, /**< Left boundary of the scanLine (inclusive). */
    (int) right, /**< Right boundary of the scanLine (exclusive). */
  });

  void draw() const;
  void clear();
  bool isValid() const,

  (std::vector<int>) fullResY, /**< All heights for a full resolution scan */
  (std::vector<HorizontalLine>) lowResHorizontalLines, /**< Description of all horizontal scanLines. */
  (std::vector<Line>) verticalLines, /**< Description of all vertical scanLines. */
  (int)(0) fieldLimit, /**< Upper bound for all scanLines (exclusive). */
  (unsigned)(0) lowResStart, /**< First index of low res grid. */
  (unsigned)(1) lowResStep, /**< Steps between low res grid lines. */
});

inline ScanGrid::Line::Line(int x, int yMin, int yMax, unsigned lowResYMaxIndex, unsigned yMaxIndex) :
  x(x), yMin(yMin), yMax(yMax), lowResYMaxIndex(lowResYMaxIndex), yMaxIndex(yMaxIndex)
{}

inline ScanGrid::HorizontalLine::HorizontalLine(int y, int left, int right) :
  y(y), left(left), right(right)
{}
