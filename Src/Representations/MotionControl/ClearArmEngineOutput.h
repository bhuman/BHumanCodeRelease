#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * Represents the output of the ClearArmEngine.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
STREAMABLE_WITH_BASE(ClearArmEngineOutput, ArmJointRequest,
{,
  (ENUM_INDEXED_ARRAY(bool, Arms::Arm)) cleared,
  (ENUM_INDEXED_ARRAY(bool, Arms::Arm)) providingOutput,
});
