/**
 * @file DamageConfiguration.h
 * Provides data about disabling some functions because of hardware failures.
 *
 * @author Benjamin Markowsky
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DamageConfiguration,
{,
  (bool)(false) weakLeftLeg,
  (bool)(false) weakRightLeg,
  (bool)(false) usLDefect,
  (bool)(false) usRDefect,
  (bool)(false) leftFootBumperDefect,
  (bool)(false) rightFootBumperDefect,
});
