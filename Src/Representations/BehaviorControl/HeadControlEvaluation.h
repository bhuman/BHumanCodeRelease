/**
 * @author <a href="mailto:andisto@tzi.de">Andreas Stolpmann</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(HeadControlEvaluation,
{,
  (unsigned)(0) ballCount,
  (unsigned)(0) goalPostCount, // Does not include total goals posts
  (unsigned)(0) totalGoalCount,
  (unsigned)(0) centerCircleCount,
  (unsigned)(0) lineCount,
  (unsigned)(0) lineIntersectionCount,
  (unsigned)(0) oppRobotCount,
  (unsigned)(0) ownRobotCount,
  (unsigned)(0) unkownRobotCount,
});
