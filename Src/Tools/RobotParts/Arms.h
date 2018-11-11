#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/BHMath.h"

namespace Arms
{
  ENUM(Arm,
  {,
    left,
    right,
  });

  static const unsigned bothArmsEnumSet = bit(left) | bit(right);
}
