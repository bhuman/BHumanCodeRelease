/**
 * @file GlobalFieldCoverage.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <vector>

STREAMABLE(GlobalFieldCoverage,
{
  void draw() const;

  /**
   * Return the timestamp when the position was last seen.
   *
   * @return timestamp When the position was last seen
   */
  unsigned timeWhenLastSeen(const Vector2f& positionOnField, const FieldDimensions& theFieldDimensions) const;

  STREAMABLE(Cell,
  {
    Cell() = default;
    Cell(const int coverage, unsigned timestamp,
         const float positionOnFieldX, const float positionOnFieldY,
         const float cellLengthX, const float cellLengthY);

    const Vector2f polygon[4], // Used for drawing

    (int)(0) coverage, /* Rating for the cell */
    (unsigned)(0) timestamp, /* Timestamp when the cell was last seen */
    (Vector2f) positionOnField, /* Position of the cell in field coordinates */
  }),

  (int)(0) meanCoverage,
  (std::vector<Cell>) grid,

  (int)(18) numOfCellsX, /* Number of cells in x direction */
  (int)(12) numOfCellsY, /* Number of cells in y direction */
  (float) cellLengthX, /* Length of a cell in x direction */
  (float) cellLengthY, /* Length of a cell in y direction */
});
