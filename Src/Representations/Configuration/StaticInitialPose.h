/**
 * @file StaticInitialPose.h
 *
 * Sets the initial Pose on field for test purpose
 *
 * @author Tim Haß
 * @author Nicole Schrader
 */

#pragma once

#include "Tools/Math/Pose2f.h"

STREAMABLE(StaticInitialPose,
{,
  (bool) isActive,
  (Pose2f) staticPoseOnField,
});
