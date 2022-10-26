/**
 * @file FilteredCurrent.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "RobotParts/Joints.h"

STREAMABLE(FilteredCurrent,
{,
  (ENUM_INDEXED_ARRAY(int, Joints::Joint)) currents, /**< The filtered currents of the joints. */
  (Joints::Joint)(Joints::lHipPitch) legJointWithHighestCurrent, /**< The leg joint with the highest current value. */
  (bool)(false) isValid, /**< Is the current filtering valid? */
  (bool)(false) legMotorMalfunction, /**< Is a motor malfunction in the legs detected? */
});
