/**
 * @file ArmPosition.h
 *
 * Declaration of struct ArmPosition.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Math/Pose3f.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
/**
 * @struct ArmPosition
 */
STREAMABLE(ArmPosition,
{
  STREAMABLE(Position,
  {,
    (Pose3f) handPose,
    (Vector3f) handPosition,
    (Vector3f) elbowPosition,
  }),

  (ENUM_INDEXED_ARRAY(Position, (Arms) Arm)) positions,
});
