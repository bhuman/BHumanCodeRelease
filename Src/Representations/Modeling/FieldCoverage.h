/**
* @file FieldCoverage.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <array>

STREAMABLE(FieldCoverage,
{
  static constexpr int numOfCellsX = 18;
  static constexpr int numOfCellsY = 12;

  STREAMABLE(GridLine,
  { ,
    (int) y,
    (std::array<unsigned, numOfCellsX>)({}) timestamps,
    (std::array<unsigned, numOfCellsX>)({}) values,
  });

  GridLine& operator[](const int y);
  const GridLine& operator[](const int y) const,

  (Vector2f) (Vector2f::Zero()) ballDropInPosition,
  (std::array<GridLine, numOfCellsY>) lines,
});

STREAMABLE(FieldCoverageLineCompressed,
{
  FieldCoverageLineCompressed() = default;
  FieldCoverageLineCompressed(const FieldCoverage::GridLine& line);
  operator FieldCoverage::GridLine() const,

  (char) y,
  (int) baseTime,
  (std::array<short, FieldCoverage::numOfCellsX>)({}) timestamps,
  (std::array<short, FieldCoverage::numOfCellsX>)({}) values,
});