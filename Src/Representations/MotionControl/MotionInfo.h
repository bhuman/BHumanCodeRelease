/**
 * @file MotionInfo.h
 *
 * Description of currently executed motion
 */

#pragma once

#include "MotionRequest.h"
#include "Tools/Debugging/DebugDrawings3D.h"

/**
 * @struct MotionInfo
 * The executed motion request and additional information about the motions which are executed by the Motion thread.
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
            specialActionRequest.specialAction == SpecialActionRequest::standHigh);
  }

  /** Helper method to avoid long and faulty expressions in some modules
   * @return true, if the MotionInfo is about a motion that performs a kick
   */
  bool isKicking() const
  {
    return motion == MotionRequest::kick ||
           (motion == MotionRequest::walk && walkRequest.walkKickRequest.kickType != WalkKicks::none);
  }

  void draw() const
  {
    DEBUG_DRAWING3D("representation:MotionInfo:upcomingOdometryOffset", "robot")
    {
      LINE3D("representation:MotionInfo:upcomingOdometryOffset", 0, 0, -210,
             upcomingOdometryOffset.translation.x(), upcomingOdometryOffset.translation.y(), -210, 2, ColorRGBA::red);
      Vector2f dir = Vector2f(50.f, 0.f);
      LINE3D("representation:MotionInfo:upcomingOdometryOffset",
             upcomingOdometryOffset.translation.x(), upcomingOdometryOffset.translation.y(), -210,
             upcomingOdometryOffset.translation.x() + dir.x(), upcomingOdometryOffset.translation.y() + dir.y(), -210, 2, ColorRGBA::blue);
      dir.rotate(upcomingOdometryOffset.rotation);
      LINE3D("representation:MotionInfo:upcomingOdometryOffset",
             upcomingOdometryOffset.translation.x(), upcomingOdometryOffset.translation.y(), -210,
             upcomingOdometryOffset.translation.x() + dir.x(), upcomingOdometryOffset.translation.y() + dir.y(), -210, 2, ColorRGBA::blue);
    }
  },

  (bool)(false) isMotionStable, /**< If true, the motion is stable, leading to a valid torso / camera matrix. */
  (Pose2f) upcomingOdometryOffset, /**< The minimum remaining odometry offset until the robot can come to a full stop. */
});
