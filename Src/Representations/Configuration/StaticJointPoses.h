/**
 * @file StaticJointPoses.h
 * This representation provides some useful static joint positions
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Angle.h"
#include "RobotParts/Joints.h"
#include "Streaming/Enum.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(StaticJointPoses,
{
  ENUM(StaticJointPoseName,
  {,
    sit,
    sitBack,
    sitFront,
    sitFrontGetUp,
    sitBackAfterFall,
    sitFrontAfterFall,
  }),

  (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(Angle, Joints::Joint), StaticJointPoses::StaticJointPoseName)) pose,
});
