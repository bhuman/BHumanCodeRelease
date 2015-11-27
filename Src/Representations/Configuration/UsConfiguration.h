#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * The class represents information about a single sonar sensor,
 * i.e. its 2f pose in the robot's torso and its opening angle.
 */
STREAMABLE_WITH_BASE(UsSensorInfo, Pose2f,
{,
  (Angle) openingAngle,
});

STREAMABLE(UsConfiguration,
{,
  (float) min,
  (float) max,
  (UsSensorInfo) leftToLeft,
  (UsSensorInfo) rightToRight,
  (UsSensorInfo) leftToRight,
  (UsSensorInfo) rightToLeft,
});
