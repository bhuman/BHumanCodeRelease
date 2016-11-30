/**
 * @file MotionInfo.h
 *
 * Description of currently executed motion
 */

#pragma once

#include "MotionRequest.h"

/**
 * @struct MotionInfo
 * The executed motion request and additional information about the motions which are executed by the Motion process.
 */
STREAMABLE_WITH_BASE(MotionInfo, MotionRequest,
{
  /** Helper method to avoid long and faulty expressions in many modules
   * @return true, if the MotionInfo is about a motion that equals standing
   */
  bool isStanding() const
  {
    return motion == MotionRequest::stand ||
    (motion == MotionRequest::specialAction &&
     (specialActionRequest.specialAction == SpecialActionRequest::stand || specialActionRequest.specialAction == SpecialActionRequest::standHigh));
  },
  
  (bool)(false) isMotionStable, /**< If true, the motion is stable, leading to a valid torso / camera matrix. */
  (Pose2f) upcomingOdometryOffset, /**< The remaining odometry offset for the currently executed motion. */
});
