/**
* @file GlobalFieldCoverage.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
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
    Cell(const unsigned timestamp, const unsigned value,
         const float positionOnFieldX, const float positionOnFieldY,
         const float cellLengthX, const float cellLengthY);

    Vector2f polygon[4], // Used for drawing

    (unsigned) (0) timestamp,
    (unsigned) (0) value,
    (Vector2f) positionOnField,
  });

  void draw() const,

  (bool) (false) newDataThisFrame,
  (std::vector<Cell>) grid,
});
