/**
 * @file FilteredCurrent.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/RobotParts/Joints.h"

STREAMABLE(FilteredCurrent,
{,
  (ENUM_INDEXED_ARRAY(int, Joints::Joint)) currents, /**< The filtered currents of the joints. */
  (bool)(false) isValid, /**< Is the current filtering valid? */
  (bool)(false) legMotorMalfunction, /**< Is a motor malfunction in the legs detected? */
});
