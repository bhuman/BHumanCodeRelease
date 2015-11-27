#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * Contains parameters that are shared between several motion modules.
 */
STREAMABLE(MotionSettings,
{,
  (float)(262.0f) comHeight, /**< Height of the com above the ground when walking/standing/kicking. */
});
