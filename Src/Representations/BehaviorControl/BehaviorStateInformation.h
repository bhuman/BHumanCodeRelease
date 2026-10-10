/**
 * @file BehaviorStateInformation.h
 *
 * Contains information about behavior state switches of the robot.
 * Used when transitions fail and need to be communicated to the user.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(BehaviorStateInformation,
{,
  (bool)(false) transitionFailed,
  (bool)(false) remoteControlActive,
});
