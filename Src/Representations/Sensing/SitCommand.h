/**
 * @file Representations/Sensing/SitCommand.h
 *
 * Declaration of struct SitCommand.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct SitCommand
 *
 * A struct that represents the current human command to let the robot sit down or not.
 */
STREAMABLE(SitCommand,
{
  ENUM(Command,
  {,
    sitExclamationMark,
    playExclamationMark,
  }),

  (Command)(playExclamationMark) command,
  (float)(-1.f) changing, /** < if 1.f is reached the state will change */
  (bool)(false) gettingCommanded,
});
