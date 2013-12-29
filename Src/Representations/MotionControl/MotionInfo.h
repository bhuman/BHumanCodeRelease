/**
* @file MotionInfo.h
* Definition of class MotionInfo.
* @author Martin Lötzsch
*/

#pragma once

#include "MotionRequest.h"

/**
* @class MotionInfo
* The executed motion request and additional information about the motions which are executed by the Motion process.
*/
STREAMABLE_WITH_BASE(MotionInfo, MotionRequest,
{,
  (bool)(false) isMotionStable, /**< If true, the motion is stable, leading to a valid torso / camera matrix. */
  (Pose2D) upcomingOdometryOffset, /**< The remaining odometry offset for the currently executed motion. */
});
