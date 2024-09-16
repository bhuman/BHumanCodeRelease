/**
 * @file WalkKickStep.h
 * This file contains information about the in walk kick
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Pose2f.h"
#include "Tools/Motion/WalkKickType.h"
#include "Tools/Motion/WalkKickVariant.h"

STREAMABLE(WalkKickStep,
{
  // Ensure currentKickVariant is always set
  WalkKickStep()
  {
    currentKickVariant = WalkKickVariant();
  }

  STREAMABLE(LastStepInfo,
  {,
    (bool)(false) nextIsLeftPhase, /**< After the last phase followed a left phase. */
    (bool)(false) wasLastPhaseLeftPhase, /**< Last motion phase was a left phase. */
    (bool)(false) wasLastPhaseInWalkKick, /**< Last motion phase was a InWalkKick. */
    (Pose2f) leftStartOffset, /** Start position of the left foot. */
    (Pose2f) rightStartOffset, /** Start position of the right foot. */
    (Pose2f) lastExecutedStep, /**< Last executed step size. */
  });

  ENUM(InterpolationType,
  {,
    normal,
    linear,
    sinusMaxToZero,
    cosinusZeroToMax,
  });

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

  // After a step finished, shall the parameters of the WalkPhase be overridden, based on the measured or requested joint angles?
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
    (Vector2f)(Vector2f(1.f, 1.f)) speedUpSwing, /**< Interpolation for the swing foot speed up. */
    (bool)(false) holdXSwingTarget, /**< X translation shall not change for the swing foot. */
    (bool)(false) holdXSupportTarget, /**< X translation shall not change for the swing foot. */
    (bool)(false) reachedWaitPosition, /**< Step keyframe finished? */
    (WalkKickStep::InterpolationType)(WalkKickStep::InterpolationType::normal) interpolationType, /**< Interpolation type for the swing foot. */
  }),

  (std::vector<StepKeyframe>) keyframe, /**< Step keyframe list.*/
  (std::vector<LongKickParams>) longKickParams, /**< Joint offset list.*/
  (float)(1.f) increaseSwingHeightFactor, /**< Swing height factor.*/
  (float)(0.f) reduceSwingFootHeight, /**< Swing height is reduced by this amount at the end of the step.*/
  (int)(0) numOfBalanceSteps, /**< Number of steps for additional balancing (if 1, then only this step of the InWalkKick is balanced).*/
  (bool)(false) useSlowSupportFootHeightAfterKickInterpolation, /**< Interpolate the support foot height slower back to 0 in the next walking step. */
  (float)(0.f) useSlowSwingFootHeightInterpolation, /**< Interpolate the swing foot height as if it was a large side step with this side step size. */
  (OverrideFoot)(OverrideFoot::none) overrideOldSwingFoot, /**< Override the previous swing foot position variables based on the last measured/requested angles. */
  (OverrideFoot)(OverrideFoot::none) overrideOldSupportFoot, /**< Override the previous support foot position variables based on the last measured/requested angles. */
  (WalkKicks::Type)(WalkKicks::none) currentKick, /**< Name of the currently executed walk kick. */
  (bool)(false) useLastKeyframeForSupportFootXTranslation, /**< Shall the support foot use the interpolation of only the last keyframe? */
  (bool)(false) usedWalkDelay, /**< A walk delay was used before the kick. */
  (std::optional<WalkKickVariant>) currentKickVariant, /**< Current kick information. Used to updated the kick. */
  (WalkKickStep::LastStepInfo) lastStepInfo, /**< Information about the previous motion phase. TODO move into currentKickVariant */
  (bool)(false) isReplayWalkRequest, /**< Flag for when replaying walk steps. This prevents updating the kick after creating. */
});
