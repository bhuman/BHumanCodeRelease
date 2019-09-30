/**
 * @file MixedTeamHeader.h
 *
 * This file declares the Mixed Team header.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

#define MIXED_TEAM_HEADER_STRUCT_HEADER "B&BH"

STREAMABLE(MixedTeamHeader,
{
  /**
   * Returns the size of this struct when it is written.
   * @return The size of ...
   */
  static int sizeOfMixedTeamHeader();

  /**
   * Converts this struct for communication usage.
   * @param data Pointer to dataspace,
   *        THIS SHOULD BE AT LEAST AS BIG AS MixedTeamHeader::sizeOfMixedTeamHeader()
   * -asserts: writing sizeOfMixedTeamHeader() bytes
   */
  void write(void* data) const;

  /**
   * Reads the message from data.
   * @param data The message.
   * @return Whether the header and the versions are convertible.
   */
  bool read(const void* data),

  (bool)(false) isPenalized, /**< Whether the robot is penalized. */
  (bool)(false) wantsToPlayBall, /**< Whether this robot is goal keeper and wants to play the ball. */
  (int)(-1) someBehaviorNumber, /**< 0-2: Back, 3-5: Center, 6-8: Front */
});
