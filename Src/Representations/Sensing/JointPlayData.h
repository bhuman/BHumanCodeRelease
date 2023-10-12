/**
 * @file JointPlayData.h
 * This representation holds information about the play of each joint.
 * @author Philip Reichenberg
 */

#pragma once

#include "RobotParts/Joints.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(JointPlayData,
{,
  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) play,
});
