#pragma once

#include "Representations/Infrastructure/JointRequest.h"

STREAMABLE_WITH_BASE(DmpKickEngineOutput, JointRequest,
{,
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
});
