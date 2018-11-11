/**
 * @file GlobalFieldCoverage.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <vector>

STREAMABLE(GlobalFieldCoverage,
{
  STREAMABLE(Cell,
  {
    Cell() = default;
    Cell(const int coverage, unsigned timestamp,
         const float positionOnFieldX, const float positionOnFieldY,
         const float cellLengthX, const float cellLengthY);

    const Vector2f polygon[4], // Used for drawing

    (int)(0) coverage,
    (unsigned)(0) timestamp,
    (Vector2f) positionOnField,
  });

  void draw() const,

  (int)(0) meanCoverage,
  (std::vector<Cell>) grid,
});
