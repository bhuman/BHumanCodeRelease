/*
 * GlobalFieldCoverage.h
 *
 * Definition of a class representing the global field coverage among all robots.
 * A part of a field is considered to be 'covered' if at least one robot looks at it.
 *
 *      Author: Felix Wenk
 */

#pragma once

#include "Representations/Modeling/FieldCoverage.h"

STREAMABLE(GlobalFieldCoverage,
{
public:
  STREAMABLE(Grid,
  {
  public:
    unsigned char coverage(int index, unsigned time) const;
    void setCoverage(int index, unsigned time, unsigned char coverage),

    /* cells[i] is the timestamp when the cell has been looked at the last time by one robot. */
    (unsigned[FieldCoverage::GridInterval::xSteps * FieldCoverage::GridInterval::ySteps]) cells,

    // Initialization
    memset(cells, 0, sizeof(cells));
  }),

  (Grid) grid,
  (int)(0) worstCoveredCellIndex,
  (int)(0) threshold, /**< The coverage value above which a cell is considered to be covered. */
  (Vector2<>) patrolTarget, /**< Target the robot should try to approach while patrolling. */
  (bool)(false) patrolTargetValid, /**< True if the patrol target is valid, otherwise false. */
});
