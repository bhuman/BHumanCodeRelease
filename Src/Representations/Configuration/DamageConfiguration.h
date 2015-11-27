/**
 * @file DamageConfiguration.h
 * Provides data about disabling some functions because of hardware failures.
 *
 * @author Benjamin Markowsky
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DamageConfigurationBody,
{,
  (bool)(false) weakLeftLeg,
  (bool)(false) weakRightLeg,
  (bool)(false) usLDefect,
  (bool)(false) usRDefect,
  (bool)(false) leftFootBumperDefect,
  (bool)(false) rightFootBumperDefect,
  (bool)(false) noFieldGenuflect,
  (bool)(false) noBackwardKick,
});

STREAMABLE(DamageConfigurationHead,
{,
  (bool)(false) audioChannel0Defect,
  (bool)(false) audioChannel1Defect,
});
