/**
 * @file RobotStatus.h
 *
 * This file declares a struct that contains the status as a robot as used in team messages.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Settings.h"
#include "Tools/Streams/AutoStreamable.h"
#include <array>

STREAMABLE(RobotStatus, COMMA BHumanCompressedMessageParticle<RobotStatus>
{
  static constexpr std::size_t numOfSequenceNumbers = Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1,

  (bool)(false) isPenalized, /**< Whether the robot is penalized. */
  (bool)(true) isUpright, /**< Whether the robot is upright. */
  (bool)(true) hasGroundContact, /**< Whether the robot has ground contact. */
  (unsigned)(0) timeWhenLastUpright, /**< The timestamp when the robot last was upright. */
  (unsigned)(0) timeOfLastGroundContact, /**< The timestamp when the robot last had ground contact. */
  (std::array<signed char, numOfSequenceNumbers>) sequenceNumbers, /**< The sequence numbers of the last received message per teammate (or my own). */
});
