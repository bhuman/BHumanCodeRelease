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
  (bool) isActive,            /**< If the module is activ. */
  (bool)(false) jump,         /**< Only Simulator! Whether the own position was changed this moment. */
  (Pose2f) staticPoseOnField, /**< The Pose to which will be switched */
});
