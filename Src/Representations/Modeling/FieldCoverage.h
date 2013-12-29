
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Vector2.h"
#include <cstring>

STREAMABLE(FieldCoverage,
{
public:
  STREAMABLE(GridInterval,
  {
  public:
    static const int xSteps = 12;
    static const int ySteps = 8;
    static const int intervals = 3; /* Number of intervals into the coverage grid is divided up. */
    static const int intervalSize = xSteps* ySteps / intervals;
    static const int maxCoverage = 255;
    static const unsigned tick = 300; /* Milliseconds one coverage tick is worth. */

    static bool assertIntervalSize() { return xSteps * ySteps % intervals == 0; }
    void nextInterval() { interval = (interval + 1) % intervals; }

    static Vector2<int> index2CellCoordinates(int idx) { return Vector2<int>(idx / ySteps, idx % ySteps); }
    static int cellCoordinates2Index(unsigned x, unsigned y) { return x * ySteps + y; },

    (unsigned)(0) timestamp, /* The timestamp when this grid has been updated. The coverage values are relative to this timestamp. */
    /* Interval of the grid currently represented.
     * Interval i represents cells from [i * (xSteps*ySteps/intervals), (i+1) * (xSteps*ySteps/intervals) - 1].
     * interval < intervals must always hold. */
    (unsigned char)(0) interval,
    (unsigned char[intervalSize]) cells,

    // Initialization
    memset(cells, 0, xSteps * ySteps / intervals);
  });

  STREAMABLE(Target,
  {
  public:
    Target(float x, float y, unsigned short coverage = 0, bool valid = true);
    Target(const Vector2<>& target, unsigned short coverage = 0, bool valid = true),

    (Vector2<>) target,
    (unsigned short)(0) coverage,
    (bool)(false) isValid, /* True if this is a valid target. The coordinates and coverage should be ignored if valid is false. */
  });

  unsigned char coverage(int cellIdx, unsigned time) const,

  (Target) worstNoTurnHalfRangeTarget, /* Worst covered cell in relative coordinates which is closer than the half the fieldcoverage range. */
  (Target) worstNoTurnRangeTarget, /* Worst covered cell in relative coordinates which is 'close' and for which you do not have to turn around. */
  (Target) worstNoTurnTarget, /* Worst covered cell on the field in relative coordinates for which you do not have to turn around. */
  (Target) worstTarget, /* Worst covered cell on the field in field coordinates. */
  (unsigned[GridInterval::xSteps* GridInterval::ySteps]) cells, /* cells[i] contains the last-seen timestamp of the i-th cell. */
  (float)(0) mean, /* The mean coverage value. */
  (float)(0) stddev, /* The variance of the coverage values.. */
  (bool)(false) throwIn, /* True if the field coverage has been modified because the ball has been thrown in. */
});
