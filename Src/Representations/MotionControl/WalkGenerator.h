/**
 * @file WalkGenerator.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Function.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"

STREAMABLE(WalkGenerator,
{
  ENUM(WalkMode,
  {,
    speedMode, /**< Speed in mm/s and radians/s. */
    stepSizeMode, /**< Step size in mm and radians (speedScale is ignored). */
    targetMode, /**< Walk to given relative position in mm and radians. */
  });

  /**
   * Initializes the generator. Must be called whenever the control is returned to this module after
   * another one was responsible for creating the motions. Must also be called once after creation.
   */
  FUNCTION(void()) reset;

  /**
   * Calculates a new set of joint angles to let the robot walk or stand. Must be called every 10 ms.
   * @param speed The speed or step size to walk with. If everything is zero, the robot stands.
   * @param target The target to walk to if in target mode.
   * @param walkMode How are speed and target interpreted?
   * @param getKickFootOffset If set, provides an offset to add to the pose of the swing foot to
   *                          create a kick motion. It must be suited for the foot that actually is
   *                          the swing foot.
   */
  FUNCTION(void(const Pose2f& speed, const Pose2f& target, WalkMode walkMode,
                const std::function<Pose3f(float phase)>& getKickFootOffset)) calcJoints,

  (JointRequest) jointRequest, /**< The calculated joint angles. */
  (Pose2f) odometryOffset, /**< The relative motion in this frame is returned here (in mm and radians). */
  (Pose2f) upcomingOdometryOffset, /**< The minimum remaining odometry offset until the robot can come to a full stop. */
  (Pose2f)(45_deg, 100.f, 100.f) maxSpeed, /**< The maximum speed possible. */
  (Pose2f) speed, /**< The average speed during the current step in mm/s and radians/s. */
  (float)(0.f) stepDuration, /**< The expected duration of the current step (in s). */
  (float) t, /**< Current time in the walk cycle (in s). */
  (bool)(false) isLeftPhase, /**< Is the left foot swinging? */
  (bool)(false) forceStand, /**< Shall a stand be forced, to prevent that the robot is stuck in a loop of emergency steps?. */
});
