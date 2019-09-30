#pragma once

#include "Representations/Infrastructure/JointRequest.h"

/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
STREAMABLE_WITH_BASE(SupersedeArmMotionEngineOutput, ArmJointRequest,
{
  STREAMABLE(Arm,
  {,
    (bool)(false) isSuperseding,
  }),

  (ENUM_INDEXED_ARRAY(Arm, Arms::Arm)) arms,
});
