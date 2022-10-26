#pragma once

#include "RobotParts/Joints.h"
#include "Streaming/EnumIndexedArray.h"
#include "Math/Range.h"

STREAMABLE(JointLimits,
{
  JointLimits(),

  (ENUM_INDEXED_ARRAY(Rangea, Joints::Joint)) limits, /**< Information on the calibration of all joints. */
});

inline JointLimits::JointLimits()
{
  limits.fill(Rangea(150_deg));
}
