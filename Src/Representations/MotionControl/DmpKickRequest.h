#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(DmpKickRequest,
{
  ENUM(DmpKickType,
  {,
    Dynamic,
    Imitated,
    BalanceOnly,
    Learn,
  });
  ,
  (bool)(true) leftSupport,/**<Whether the left or the right leg should be support*/
  (DmpKickType)(Imitated) type,
  (bool)(true) trackBall, /**<If true the current ball percept will be used as kick target */
  (Vector3f)(Vector3f(1500.0f, 0.0f, 0.0f)) kickSpeed, /**<Speed of the kickfoot */
});
