#pragma once

#include <vector>
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyPoseRequest.h"
#include "Tools/RobotParts/Arms.h"

/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
STREAMABLE_WITH_BASE(PointAtEngineOutput, JointRequest,
{,
});
