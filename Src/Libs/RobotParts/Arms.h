#pragma once

#include "Math/BHMath.h"
#include "Streaming/Enum.h"

namespace Arms
{
  ENUM(Arm,
  {,
    left,
    right,
  });

  static const unsigned bothArmsEnumSet = bit(left) | bit(right);
}
