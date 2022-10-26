#pragma once

#include "Math/Angle.h"

namespace SensorData
{
  // See \c OutTextRaw::writeAngle.
  constexpr Angle off = 10000.f; /**< Special value that indicates that a sensor is turned off. */
  constexpr Angle ignore = 20000.f; /**< Special value that indicates that a sensor is ignored, and uses the previous value. */
}
