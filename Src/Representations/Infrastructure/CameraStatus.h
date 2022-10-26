/**
 * Defines a representation that contains whether both cameras are ok.
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(CameraStatus,
{,
  (bool)(true) ok,
});
