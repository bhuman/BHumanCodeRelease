/**
 * @file WalkKickStep.h
 * This file contains information about the in walk kick
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Motion/WalkKickType.h"

STREAMABLE(WalkKickStep,
{
  // Parameters for specific joint offsets during an InWalkKick
  STREAMABLE(LongKickParams,
  {,
    (Joints::Joint) joint, // the joint
    (Angle) offset, // offset amount
    (float) minRatio, // start ratio for the sinus interpolation
    (float) middleRatio, // max value for the sinus interpolation
    (float) maxRatio, // 0 end position for the sinus interpolation
    (float) minimumRatio, // minRatio has at least this value, when shifted
    (int) shiftByKeyFrame, // shift the ratio position based on this keyframe
  });

  // After a step finished, shall the paramters of the WalkPhase be overridden, based on the measured or requested joint angles?
  ENUM(OverrideFoot,
  {,
    none,
    measured,
    request,
  });

  STREAMABLE(StepKeyframe,
  {,
    (Pose2f)(Pose2f()) stepTarget, /**< Planned step target. */
    (Pose2f)(Pose2f()) stepTargetSwing, /**< Planned step target for the swing foot. */
    (Pose2f)(Pose2f()) stepTargetConverted, /**< Converted step target, based on the rotation. */
    (Pose2f)(Pose2f()) stepTargetSwingConverted, /**< Converted step target for the swing foot, based on the rotation. */
    (float)(0.5f) stepRatio, /**< Duration ratio for this step keyframe */
    (float)(1.f) speedUpSwing, /**< Interpolation for the swing foot speed up. */
    (bool)(false) holdXSwingTarget, /**< X translation shall not change for the swing foot. */
    (bool)(false) holdXSupportTarget, /**< X translation shall not change for the swing foot. */
    (bool)(false) reachedWaitPosition, /**< Step keyframe finished? */
  }),

  (std::vector<StepKeyframe>) keyframe, /**< Step keyframe list.*/
  (std::vector<LongKickParams>) longKickParams, /**< Joint offset list.*/
  (float)(1.f) increaseSwingHeightFactor, /**< Swing height factor.*/
  (float)(0.f) reduceSwingFootHeight, /**< Swing height is reduced by this amount at the end of the step.*/
  (int)(0) numOfBalanceSteps, /**< Number of steps for additional balancing (if 1, then only this step of the InWalkKick is balanced).*/
  (bool)(false) useSlowFootHeightInterpolation, /**< Interpolate the support foot height slower back to 0 in the next walking step. */
  (OverrideFoot)(OverrideFoot::none) overrideOldSwingFoot, /**< Override the previous swing foot position variables based on the last measured/requested angles. */
  (OverrideFoot)(OverrideFoot::none) overrideOldSupportFoot, /**< Override the previous support foot position variables based on the last measured/requested angles. */
  (WalkKicks::Type)(WalkKicks::none) currentKick, /**< Name of the currently executed walk kick. */
});
