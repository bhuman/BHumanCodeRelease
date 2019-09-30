/**
 * @file EnergySaving.h
 * Current offsets, to reduce the current in the joints as much as possible
 * @author Philip Reichenberg
 */

#pragma once
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/RobotParts/Joints.h"

STREAMABLE(EnergySaving,
{
  EnergySaving(),

  (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) offsets,
});

inline EnergySaving::EnergySaving()
{
  offsets.fill(0_deg);
}
