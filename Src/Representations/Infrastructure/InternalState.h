/**
 * @file InternalState.h
 *
 * This file defines a representation that represents the internal state
 * that is written to a file and read by the DeployDialog.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(InternalState,
{,
  (float)(0.f) batteryLevel,
  (float)(0.f) maxTemperature,
});
