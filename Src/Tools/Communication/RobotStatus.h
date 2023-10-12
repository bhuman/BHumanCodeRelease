/**
 * @file RobotStatus.h
 *
 * This file declares a struct that contains the status as a robot as used in team messages.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Communication/BHumanMessageParticle.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(RobotStatus, COMMA BHumanCompressedMessageParticle<RobotStatus>
{,
  (bool)(true) isUpright, /**< Whether the robot is upright. */
  (unsigned)(0) timeWhenLastUpright, /**< The timestamp when the robot last was upright. */
});
