/**
 * @file HeadMotionInfo.h
 *
 * This file declares a struct that represents the status of the head motion.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(HeadMotionInfo,
{,
  (bool)(false) moving, /**< Whether the head is currently moving or not. */
});
