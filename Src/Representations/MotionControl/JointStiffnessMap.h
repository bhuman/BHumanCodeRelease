/**
 * @file Representations/MotionControl/JointStiffnessMap.h
 * @author Philip Reichennerg
 */

#pragma once
#include "Math/Eigen.h"
#include "Streaming/Enum.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(JointStiffnessMapping,
{
  ENUM(Type,
  {,
    regular,
    low,
  }),

  (Type)(JointStiffnessMapping::regular) type,
});
