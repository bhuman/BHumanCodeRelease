/**
 * @file JointSpeedController.h
 * This file implements a controller for the joints.
 * The idea is to make the movement of the joints smarter in a specific way.
 * If the CoM of the robot is too far forward, it makes no sense that the pitch joints
 * are allowed to let the robot tilt even further. Some for the other direction.
 * Therefore such changes are reduced to a minimum.
 *
 * For example, lets assume the robot is walking forward and is tilted forward, with a resulting CoM near the tip of the toe.
 * The supporting foot is requested to move backwards.
 * This means:
 * - the HipPitch will move positive (good!),
 * - the Knee stays the same (good) until the end, where it will move negative (bad!)
 * - the AnklePitch will move negative (bad!)
 *
 * The controller makes sure, that the Hip, Knee and Ankle are still allowed to execute the change of direction (moving negative),
 * but the size of the change will be reduced. For example the ankle would move like -1deg per frame,
 * the controller reduces it down to -0.5deg per frame. This is enough to stabilize the walking significantly!
 *
 * Note:
 * The current min-parameters are just some first thought ones. I just wrote some down and tested some.
 * From experiments I noticed, that it is better to allow some movement than none.
 * Otherwise the adjustment either was to strong (min values are too low) and results in overbalancing, or
 * the robot starts running and lifts itself up on the tip of the toe.
 * This seems to be unharmful (nothing broke over several weeks!), but results in very large walking steps.
 * The min values can also be too large, but then it would be as if no changes was be applied.
 *
 * For the z-rotation, a direct adjustment for the HipYawPitch is a really bad idea. Because of the kinematic of the robot
 * and how this joint is build in,changes for this joint would break all calculation. I tested it and the legs would do
 * broken motions (because the translations and rotation are calculated wrong).
 * Therefor the z-Rotation is adjusted over the WalkPhase parameter turnRL, which directly controls the z-Rotation itself.
 *
 * As another side note:
 * If a running walk is ever planned, simply not allowing the ankle pitches of the support foot to move
 * and tilting the robot forward (to bring the CoM forward) would already allow the robot to walk > 600 mm/s.
 * Only the height differences of the swing and support leg at the end of each step, or a pendulum balancing,
 * need to be controlled. Because of the high walking speed, the robot likes to drift away and then just fall diagonal/sideways.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "RobotParts/Joints.h"
#include "Math/Range.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(SpeedControlParams,
{,
  // Min of the range is the min allowed rotation speed per motion frame
  // Max is added on top of the calculated max rotation speed needed in the current walking step
  (Rangea)({0, 0}) rotationSpeedHYP, /**< Rotation speeds for the z-Rotation. */
  (Rangea)({0, 0}) rotationSpeedHip, /**< Rotation speeds for the joint HipPitch. */
  (Rangea)({0, 0}) rotationSpeedAnklePitch, /**< Rotation speeds for the joint AnklePitch. */
  (Rangea)({0, 0}) rotationSpeedKnee, /**< Rotation speeds for the joint KneePitch. */
  (Rangea)({0, 0}) rotationSpeedRoll, /**< Rotation speeds for the joints HipRoll and AnkleRoll. */
  (Rangef) pitchRatioForward, /**< % position of the CoM. Used as an interpolation range for the forward errors. */
  (Rangef) pitchRatioBackward, /**< % position of the CoM. Used as an interpolation range for the backward errors. */
  (Rangef) rollRatio, /**< % position of the CoM. Used as an interpolation range for the side errors. See RobotStableState for more information. */
  (Rangea) rotationErrorRatioForwardWornOut, /**< Support sole rotation error interpolation range, for the forward case. */
  (Rangea) rotationErrorRatioForwardGood, /**< Support sole rotation error interpolation range, for the forward case. */
  (Rangea) rotationErrorRatioBackward, /**< Support sole rotation error interpolation range, for the backward case. */
});

struct JointSpeedController : ENUM_INDEXED_ARRAY(Rangea, Joints::Joint)
{
  JointSpeedController(const ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, Angle turn);

  /**
   * Update allowed position ranges
   * @param scp The parameters
   * @param currentPitchPosition The % CoM position in the x-axis of the foot area
   * @param currentRollPosition The % CoM position in the y-axis of the foot area. See RobotStableState, for more details
   * @param isLeftPhase Is the left foot the swing one?
   */
  void update(const SpeedControlParams& scp, const float currentPitchPosition, const float currentRollPosition, const bool isLeftPhase, const Angle gyroY);

  /**
   * Apply the control to the request
   * @param currentJoints The requested positions
   * @param isLeftPhase Is the left foot the swing one?
   */
  void apply(ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const bool isLeftPhase);

  /**
   * Update the parameter for the allowed position ranges, based on the rotation error of the support foot
   * @param rotationErrorMeasured The y rotation error
   * @param scp The parameters
   */
  void applySoleError(const Angle rotationErrorMeasured, const SpeedControlParams& scp, const float hardwareValue);

  /**
   * Reset the controller to the given joint positions
   * @param currentJoints The joint positions
   * @param turn The z-Rotation
   */
  void reset(const ENUM_INDEXED_ARRAY(Angle, Joints::Joint)& currentJoints, const Angle turn);

  /**
   * Update the allowed z-Rotations
   * @param scp The parameters
   * @param rotationError The y rotation error of the support foot
   * @param currentPitchPosition The % CoM position in the foot area
   * @param currentPitchPositionTorso The % CoM position in the foot area
   * @param hardwareValue Value in the range [0..1] for how bad the robot is. 0 means bad, 1 good.
   * @param gyroY The current y gyro value
   */
  void updateZRotation(const SpeedControlParams& scp, const Angle rotationError, const float currentPitchPosition, const float currentPitchPositionTorso, const float hardwareValue, const Angle gyroY);

  /**
   * Clip the z-Rotation
   * @param turn The z-Rotation
   */
  void clipZRotation(Angle& turn);

  Rangea turnRange; /**< The allowed z-Rotation range. */
  float rotationErrorRatioForward = 1.f; /**< The interpolation factor for the forward pitch rotations. */
  float rotationErrorRatioBackward = 1.f; /**< The interpolation factor for the backward pitch rotations. */
};
