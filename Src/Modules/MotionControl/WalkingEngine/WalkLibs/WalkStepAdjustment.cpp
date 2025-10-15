/**
 * @file WalkStepAdjustment.h
 * This file declares our Walk Step Adjustment. See our paper <Step Adjustment for a Robust Humanoid Walk> for more details.
 * @author Philip Reichenberg
 */

#include "WalkStepAdjustment.h"
#include "Debugging/Annotation.h"
#include "Math/Geometry.h"
#include "Math/Rotation.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include <cmath>

WalkStepAdjustment::WalkStepAdjustment() :
  swingRotYFilter(0.6f, 0.9f),
  swingRotXFilter(0.6f, 0.9f),
  kneeBalanceFilter(0.2f, 1.f)
{
  lastLeftAdjustmentX = 0.f;
  lastRightAdjustmentX = 0.f;
  highestNegativeAdjustmentX = 0.f;
  highestAdjustmentX = 0.f;
  lastLargeBackwardStep = 0;
  lastNormalStep = 0;
  reduceWalkingSpeed = 0;

  for(std::size_t i = 0; i < kneeBalanceFilter.buffer.capacity(); i++)
    kneeBalanceFilter.update(0.01_deg * (i + 1)); // set up buffer to ensure it can move knee as fast as possible
}

void WalkStepAdjustment::addBalance(Pose3f& left, Pose3f& right, const float stepTime, const float stepDuration, const Vector2f& com, const Rangef& footOffset,
                                    const Rangef& clipForwardPosition, const bool isLeftPhase, const FootSupport& footSupport,
                                    const SolePressureState& solePressureState, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                                    const int reduceWalkingSpeedStepAdjustmentSteps, const Vector2f& ball, const float clipAtBallDistanceX,
                                    const bool groundContact, const WalkStepAdjustmentParams& walkStepParams, const float forwardWalkingSpeed, const bool afterWalkKickPhase)
{
  if(walkStepParams.useAccelerations)
  {
    addBalanceWithAccelerations(left, right, stepTime, stepDuration, com, footOffset, clipForwardPosition,
                                isLeftPhase, footSupport, solePressureState, frameInfo,
                                hipRot, isStepAdjustmentAllowed, reduceWalkingSpeedStepAdjustmentSteps,
                                ball, clipAtBallDistanceX, groundContact, walkStepParams, forwardWalkingSpeed, afterWalkKickPhase);
    return;
  }

  // If
  // - the current support foot is not the correct one,
  // - the feet have no ground contact,
  // - the swing foot has ground contact and the step did not just started
  // then the feet should not get balanced.
  // current support foot and the swing foot pressure are checked, because the swing foot could be the support foot, but with very low pressure.
  // Otherwise the feet are unnecessary balanced (with robot is hold in the air by a human) or the chance, the robot might fall, is even higher

  ///////////////////////////////////
  // 1. Set allowed movement speed //
  ///////////////////////////////////
  const float stepRatio = stepTime / stepDuration;
  ASSERT(std::isfinite(ball.x()) && std::isfinite(ball.y()));
  ASSERT(std::isfinite(stepRatio));
  const float comX = com.x();
  float useMaxVelX = walkStepParams.maxVelX * motionCycleTime;
  float useMinVelX = walkStepParams.minVelX * motionCycleTime;
  float useRemoveSpeedX = walkStepParams.removeSpeedX * motionCycleTime;
  const bool wrongSupportFoot = footSupport.support < 0.f != isLeftPhase;
  const bool swingFootHasGroundContact = (isLeftPhase && solePressureState.legInfo[Legs::left].hasPressure) || (!isLeftPhase && solePressureState.legInfo[Legs::right].hasPressure);
  const Rangef desiredFootArea(walkStepParams.desiredFootAreaSmall.min, walkStepParams.desiredFootAreaSmall.max);
  if(!groundContact)
  {
    useMaxVelX = 0.f;
    useMinVelX = 0.f;
    useRemoveSpeedX = 0.f;
  }
  else if(swingFootHasGroundContact && !wrongSupportFoot)
  {
    useMaxVelX = stepRatio < 0.2f ? useMaxVelX : 0.f;
    useMinVelX = stepRatio < 0.2f ? useMinVelX : 0.f;
    useRemoveSpeedX = stepRatio < 0.2f ? useRemoveSpeedX : 0.f;
  }
  else if(stepRatio > 1.0f)
  {
    const float ratio = mapToRange(stepRatio, 1.0f, 1.35f, 1.f, 0.f);
    useMaxVelX = ratio * useMaxVelX + (1.f - ratio) * useMinVelX;
  }

  if(lastLeft == Pose3f())
  {
    lastLeft = left;
    lastRight = right;
  }

  //////////////////////////////////////////
  // Start with step adjustment algorithm //
  //////////////////////////////////////////

  // 3. copy for later
  const Pose3f originalSwing = isLeftPhase ? left : right;
  const Pose3f originalSupport = !isLeftPhase ? left : right;

  // 4. Get references to swing and support foot
  // swingAdjustment = last[Left|Right]Adjustment = delta{w_t-1} - w_t-1
  float& swingAdjustment = isLeftPhase ? lastLeftAdjustmentX : lastRightAdjustmentX;
  float& supportAdjustment = !isLeftPhase ? lastLeftAdjustmentX : lastRightAdjustmentX;
  // w_t and u_t
  Pose3f& swingFoot = isLeftPhase ? left : right;
  Pose3f& supportFoot = !isLeftPhase ? left : right;
  // bar{w_t-1} and bar{u_t-1}
  // w_t-1 abd u_t-1 are equal to bar{w_t-1} and bar{u_t-1}, when swingAdjustment is subtracted
  const Pose3f& lastSwingFoot = isLeftPhase ? lastLeft : lastRight;

  // 5. Calculate the tolerance range
  // bar{w_t-1}
  const float lastSwingX = lastSwingFoot.translation.x();

  // change of the swing foot based on the planned walk
  const float plannedChange = (swingFoot.translation.x() - (lastSwingFoot.translation.x() - swingAdjustment));

  const float maxAdjustmentAfterStop = swingAdjustment - plannedChange;
  const bool hadAdjustmentLastFrame = swingAdjustment != 0.f;

  // deltaReduced = delta{hat{w_t-1}}
  const Rangef adjustmentClip(-useMaxVelX - useRemoveSpeedX + std::min(-plannedChange, 0.f), useMaxVelX + useRemoveSpeedX + std::max(-plannedChange, 0.f));
  float deltaReduced = -adjustmentClip.limit(swingAdjustment);

  if(stepRatio >= 0.5f)
  {
    const Pose3f& lastSupportFoot = !isLeftPhase ? lastLeft : lastRight;
    const float otherDeltaReduced = (lastSupportFoot.translation.x() - lastSwingFoot.translation.x()) / 1.5f;
    if(deltaReduced < 0.f)
      deltaReduced = std::max(-swingAdjustment, std::min(deltaReduced, otherDeltaReduced));
    else if(deltaReduced > 0.f)
      deltaReduced = std::min(-swingAdjustment, std::max(deltaReduced, otherDeltaReduced));
  }

  // 2. low pass filter if feet are already adjusted, to reduce damage from noise
  if(std::abs(swingAdjustment + deltaReduced) > 0.1f)
    lowPassFilteredComX = walkStepParams.comLowPassRatio * lowPassFilteredComX + (1.f - walkStepParams.comLowPassRatio) * comX;
  else
    lowPassFilteredComX = comX;

  if(!groundContact)
    lowPassFilteredComX = 0.f;

  // currentPlannedSwingX = hat{w_t}
  const float currentPlannedSwingX = lastSwingX + plannedChange + deltaReduced;

  // Current feet area
  // For InWalkKicks, the swing foot must contain the CoM at all times to ensure stability
  const Rangef toleranceSpecificationSupport = Rangef((footOffset.min + footOffset.max) * desiredFootArea.min - footOffset.min, (footOffset.min + footOffset.max) * desiredFootArea.max - footOffset.min);
  const Rangef toleranceSpecificationSwing = walkStepParams.useFullSwingSole ? toleranceSpecificationSupport : Rangef(-footOffset.min, footOffset.max);

  float toleranceLowerLimit, toleranceUpperLimit, supportOffset = 0.f;

  // toleranceLowerLimit = Delta_min; toleranceUpperLimit = Delta_max;
  toleranceLowerLimit = std::min(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecificationSupport.min;
  toleranceUpperLimit = std::max(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecificationSupport.max;
  if(!walkStepParams.useFullSwingSole)
  {
    if(currentPlannedSwingX + toleranceSpecificationSwing.min > toleranceLowerLimit)
    {
      const float oldLimit = toleranceLowerLimit;
      toleranceLowerLimit = currentPlannedSwingX + toleranceSpecificationSwing.min;
      supportOffset = toleranceLowerLimit - oldLimit; // should be a positive sign
    }
    if(currentPlannedSwingX + toleranceSpecificationSwing.max < toleranceUpperLimit)
    {
      const float oldLimit = toleranceUpperLimit;
      toleranceUpperLimit = currentPlannedSwingX + toleranceSpecificationSwing.max;
      supportOffset = oldLimit - toleranceUpperLimit; // should be a negative sign
    }
  }
  // when the arms are on the back, the upper body is tilted forward.
  // if hipRot == 0, nothing changes
  const float walkHeight = std::max(supportFoot.translation.z(), swingFoot.translation.z());
  const float deltaMin = Vector2f(toleranceLowerLimit, walkHeight).rotate(-hipRot).x();
  const float deltaMax = Vector2f(toleranceUpperLimit, walkHeight).rotate(-hipRot).x();

  // 6. delta
  delta = isStepAdjustmentAllowed ? std::min(lowPassFilteredComX - deltaMin, 0.f) + std::max(lowPassFilteredComX - deltaMax, 0.f) : 0.f;

  // hat{p_max}; hat{p_min}
  const float pMin = std::min(-useMinVelX, adjustmentClip.min - plannedChange) - std::max(deltaReduced, 0.f);
  const float pMax = std::max(useMinVelX, adjustmentClip.max - plannedChange) - std::min(deltaReduced, 0.f);

  // 7. Apply adjustment
  // Apply swing foot adjustment
  swingAdjustment = (currentPlannedSwingX + std::max(pMin, std::min(pMax, delta))) - swingFoot.translation.x();

  if(std::abs(swingAdjustment) < 0.01f)
    swingAdjustment = 0.f;

  if((stepRatio >= 0.5f && highestAdjustmentX != 0.f && hadAdjustmentLastFrame && std::abs(swingAdjustment) < std::abs(maxAdjustmentAfterStop) &&
      swingAdjustment * maxAdjustmentAfterStop >= 0.f))
    adjustmentStopCounter++;
  else if(adjustmentStopCounter > 2)
    adjustmentResumeCounter++;
  if(adjustmentResumeCounter > 2)
  {
    adjustmentResumeCounter = 0;
    adjustmentStopCounter = 0;
  }
  if(adjustmentStopCounter > 2)
  {
    const Rangef swingAdjustmentStopRange(std::min(0.f, maxAdjustmentAfterStop), std::max(0.f, maxAdjustmentAfterStop));
    swingAdjustment = swingAdjustmentStopRange.limit(swingAdjustment);
    if(Approx::isZero(swingAdjustment))
      adjustmentStopCounter = 0;
  }

  swingFoot.translation.x() += swingAdjustment;

  // clip to make sure the feet pose are reachable
  swingFoot.translation.x() = clipForwardPosition.limit(swingFoot.translation.x());
  swingAdjustment = swingFoot.translation.x() - originalSwing.translation.x();

  // Apply support foot adjustment
  supportAdjustment += std::min(-pMin + useRemoveSpeedX, std::max(-pMax - useRemoveSpeedX, -swingAdjustment / 2.f - supportAdjustment));
  const Rangef supportOffsetClipRange(std::min(0.f, supportOffset), std::max(0.f, supportOffset));
  supportAdjustment -= supportOffsetClipRange.limit(supportAdjustment);
  supportFoot.translation.x() += supportAdjustment;

  supportFoot.translation.x() = clipForwardPosition.limit(supportFoot.translation.x());
  supportAdjustment = supportFoot.translation.x() - originalSupport.translation.x();

  ////////////////////////////////////////////////////////////////
  // clip to make sure we do not unintentionally touch the ball //
  ////////////////////////////////////////////////////////////////
  // but if the robot would fall, do it anyway
  if(delta < 25.f && !allowTouchingTheBallForBalancing)  // Todo 25.f should be a parameter, and maybe change code to still clip, instead of allowing full adjustment after exceeding the 25 mm mark
  {
    const float clipAdjustment = std::max((supportFoot.translation.x() + std::max(ball.x() - clipAtBallDistanceX, 0.f)) - (swingFoot.translation.x() - swingAdjustment), 0.f) / 1.5f;
    const float clippedAdjustment = std::min(clipAdjustment - swingAdjustment, 0.f);
    swingAdjustment += clippedAdjustment;
    supportAdjustment -= clippedAdjustment / 2.f;
    swingFoot.translation.x() += clippedAdjustment;
    supportFoot.translation.x() -= clippedAdjustment / 2.f;
  }
  else
    allowTouchingTheBallForBalancing = true;

  // Save highest adjustments for later
  highestAdjustmentX = (swingAdjustment < 0.f ? -1.f : (swingAdjustment > 0.f ? 1.f : 0.f)) * std::max(std::abs(highestAdjustmentX), std::abs(swingAdjustment)); // save the highest value for the current step
  highestNegativeAdjustmentX = std::min(highestNegativeAdjustmentX, highestAdjustmentX);

  // Too much backward adjustment. Reduce walking speed
  if((highestNegativeAdjustmentX < -walkStepParams.unstableWalkThreshold &&  // threshold check
      frameInfo.getTimeSince(lastNormalStep) < walkStepParams.reduceWalkingSpeedTimeWindow && // only reduce if we adjusted two times backwards in the last time frame
      frameInfo.getTimeSince(lastLargeBackwardStep) < walkStepParams.reduceWalkingSpeedTimeWindow) ||
     (highestAdjustmentX > walkStepParams.unstableWalkThreshold && std::abs(highestAdjustmentX) != highestNegativeAdjustmentX)) // reduce walk speed if we adjusted a lot forward
  {
    if(reduceWalkingSpeed != reduceWalkingSpeedStepAdjustmentSteps)
      ANNOTATION("WalkingEngine", "Reduced Walking Speed!");
    reduceWalkingSpeed = reduceWalkingSpeedStepAdjustmentSteps;
  }

  lastLeft = left;
  lastRight = right;

  ///////////////////
  // Debug Drawings //
  ///////////////////
  COMPLEX_DRAWING3D("module:WalkStepAdjustment:balance")
  {
    Pose3f leftDrawing = left;
    Pose3f rightDrawing = right;
    Pose3f originLeftDrawing = isLeftPhase ? originalSwing : originalSupport;
    Pose3f originRightDrawing = !isLeftPhase ? originalSwing : originalSupport;
    const Vector2f leftShift = Vector2f(left.translation.x(), left.translation.z()).rotate(-hipRot);
    const Vector2f rightShift = Vector2f(right.translation.x(), right.translation.z()).rotate(-hipRot);
    const Vector2f leftOriginShift = Vector2f(originLeftDrawing.translation.x(), originLeftDrawing.translation.z()).rotate(-hipRot);
    const Vector2f rightOriginShift = Vector2f(originRightDrawing.translation.x(), originRightDrawing.translation.z()).rotate(-hipRot);
    leftDrawing.translation.x() = leftShift.x();
    leftDrawing.translation.z() = leftShift.y();
    rightDrawing.translation.x() = rightShift.x();
    rightDrawing.translation.z() = rightShift.y();
    originLeftDrawing.translation.x() = leftOriginShift.x();
    originLeftDrawing.translation.z() = leftOriginShift.y();
    originRightDrawing.translation.x() = rightOriginShift.x();
    originRightDrawing.translation.z() = rightOriginShift.y();

    // Debug Drawings
    const Pose3f maxLeft = (leftDrawing + Vector3f(toleranceSpecificationSupport.max, 0.f, 0.f));
    const Pose3f minLeft = (leftDrawing + Vector3f(toleranceSpecificationSupport.min, 0.f, 0.f));
    const Pose3f maxRight = (rightDrawing + Vector3f(toleranceSpecificationSupport.max, 0.f, 0.f));
    const Pose3f minRight = (rightDrawing + Vector3f(toleranceSpecificationSupport.min, 0.f, 0.f));

    const Pose3f& swingPos = isLeftPhase ? leftDrawing : rightDrawing;
    const Pose3f& supportPos = !isLeftPhase ? leftDrawing : rightDrawing;
    const Pose3f maxSwingPose = (swingPos + Vector3f(toleranceSpecificationSwing.max, 0.f, 0.f));
    const Pose3f minSwingPose = (swingPos + Vector3f(toleranceSpecificationSwing.min, 0.f, 0.f));

    LINE3D("module:WalkStepAdjustment:balance", maxLeft.translation.x(), maxLeft.translation.y() - 30.f, maxLeft.translation.z(), maxLeft.translation.x(), maxLeft.translation.y() + 30.f, maxLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minLeft.translation.x(), minLeft.translation.y() - 30.f, minLeft.translation.z(), minLeft.translation.x(), minLeft.translation.y() + 30.f, minLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxRight.translation.x(), maxRight.translation.y() - 30.f, maxRight.translation.z(), maxRight.translation.x(), maxRight.translation.y() + 30.f, maxRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minRight.translation.x(), minRight.translation.y() - 30.f, minRight.translation.z(), minRight.translation.x(), minRight.translation.y() + 30.f, minRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxSwingPose.translation.x(), maxSwingPose.translation.y() - 30.f, maxSwingPose.translation.z(), maxSwingPose.translation.x(), maxSwingPose.translation.y() + 30.f, maxSwingPose.translation.z(), 3, ColorRGBA::cyan);
    LINE3D("module:WalkStepAdjustment:balance", minSwingPose.translation.x(), minSwingPose.translation.y() - 30.f, minSwingPose.translation.z(), minSwingPose.translation.x(), minSwingPose.translation.y() + 30.f, minSwingPose.translation.z(), 3, ColorRGBA::cyan);
    LINE3D("module:WalkStepAdjustment:balance", toleranceUpperLimit, swingPos.translation.y() - 30.f, supportPos.translation.z(), toleranceUpperLimit, swingPos.translation.y() + 30.f, supportPos.translation.z(), 3, ColorRGBA::green);
    LINE3D("module:WalkStepAdjustment:balance", toleranceLowerLimit, swingPos.translation.y() - 30.f, supportPos.translation.z(), toleranceLowerLimit, swingPos.translation.y() + 30.f, supportPos.translation.z(), 3, ColorRGBA::green);

    LINE3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), originLeftDrawing.translation.x(), originLeftDrawing.translation.y(), originLeftDrawing.translation.z(), 2, ColorRGBA::orange);
    LINE3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), originRightDrawing.translation.x(), originRightDrawing.translation.y(), originRightDrawing.translation.z(), 2, ColorRGBA::orange);
    CROSS3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), 3, 3, ColorRGBA::black);
    CROSS3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), 3, 3, ColorRGBA::black);
  }

  PLOT("module:WalkStepAdjustment:Data:leftAdjustment", lastLeftAdjustmentX);
  PLOT("module:WalkStepAdjustment:Data:rightAdjustment", lastRightAdjustmentX);
}

void WalkStepAdjustment::addBalanceWithAccelerations(Pose3f& left, Pose3f& right, const float stepTime, const float stepDuration, const Vector2f& com, const Rangef& footOffset,
                                                     const Rangef& clipForwardPosition, const bool isLeftPhase, const FootSupport& footSupport,
                                                     const SolePressureState& solePressureState, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                                                     const int reduceWalkingSpeedStepAdjustmentSteps, const Vector2f&, const float,
                                                     const bool groundContact, const WalkStepAdjustmentParams& walkStepParams, const float forwardWalkingSpeed, const bool afterWalkKickPhase)
{
  // If
  // - the current support foot is not the correct one,
  // - the feet have no ground contact,
  // - the swing foot has ground contact and the step did not just started
  // then the feet should not get balanced.
  // current support foot and the swing foot pressure are checked, because the swing foot could be the support foot, but with very low pressure.
  // Otherwise the feet are unnecessary balanced (with robot is hold in the air by a human) or the chance, the robot might fall, is even higher

  ///////////////////////////////////
  // 1. Set allowed movement speed //
  ///////////////////////////////////
  const float stepRatio = stepTime / stepDuration;
  //ASSERT(std::isfinite(ball.x()) && std::isfinite(ball.y()));
  ASSERT(std::isfinite(stepRatio));
  const float comX = com.x();
  const float deltaScalingStuff = mapToRange(std::abs(delta), 0.f, 30.f, walkStepParams.minVelX, walkStepParams.maxVelX);
  float useMaxVelX = deltaScalingStuff * motionCycleTime;
  float useMinVelX = deltaScalingStuff * motionCycleTime;
  float useRemoveSpeedX = walkStepParams.removeSpeedX * motionCycleTime;
  const bool wrongSupportFoot = footSupport.support < 0.f != isLeftPhase;
  const bool swingFootHasGroundContact = (isLeftPhase && solePressureState.legInfo[Legs::left].hasPressure) || (!isLeftPhase && solePressureState.legInfo[Legs::right].hasPressure);

  float useBigMin = highestAdjustmentX < 0.f || afterWalkKickPhase ? walkStepParams.desiredFootAreaSmall.min : mapToRange(forwardWalkingSpeed, 0.f, walkStepParams.footAreaWalkSpeedScale.max, walkStepParams.desiredFootAreaSmall.min, walkStepParams.desiredFootAreaBig.min);
  float useBigMax = highestAdjustmentX > 0.f || afterWalkKickPhase ? walkStepParams.desiredFootAreaSmall.max : mapToRange(forwardWalkingSpeed, walkStepParams.footAreaWalkSpeedScale.min, 0.f, walkStepParams.desiredFootAreaBig.max, walkStepParams.desiredFootAreaSmall.max);
  useBigMax = mapToRange(stepTime, stepDuration, stepDuration + walkStepParams.overtimeRange, useBigMax, walkStepParams.desiredFootAreaForwardOvertime);
  const Rangef desiredFootArea(useBigMin, useBigMax);
  if(!groundContact)
  {
    useMaxVelX = 0.f;
    useMinVelX = 0.f;
    useRemoveSpeedX = 0.f;
  }
  else if(swingFootHasGroundContact && !wrongSupportFoot)
  {
    useMaxVelX = stepRatio < 0.2f ? useMaxVelX : 0.f;
    useMinVelX = stepRatio < 0.2f ? useMinVelX : 0.f;
    useRemoveSpeedX = stepRatio < 0.2f ? useRemoveSpeedX : 0.f;
  }

  if(lastLeft == Pose3f())
  {
    lastLeft = left;
    lastRight = right;
  }

  //////////////////////////////////////////
  // Start with step adjustment algorithm //
  //////////////////////////////////////////

  // 3. copy for later
  const Pose3f originalSwing = isLeftPhase ? left : right;
  const Pose3f originalSupport = !isLeftPhase ? left : right;

  // 4. Get references to swing and support foot
  // swingAdjustment = last[Left|Right]Adjustment = delta{w_t-1} - w_t-1
  float& swingAdjustment = isLeftPhase ? lastLeftAdjustmentX : lastRightAdjustmentX;
  float& supportAdjustment = !isLeftPhase ? lastLeftAdjustmentX : lastRightAdjustmentX;
  // w_t and u_t
  Pose3f& swingFoot = isLeftPhase ? left : right;
  Pose3f& supportFoot = !isLeftPhase ? left : right;
  // bar{w_t-1} and bar{u_t-1}
  // w_t-1 abd u_t-1 are equal to bar{w_t-1} and bar{u_t-1}, when swingAdjustment is subtracted
  const Pose3f& lastSwingFoot = isLeftPhase ? lastLeft : lastRight;

  // 5. Calculate the tolerance range
  // bar{w_t-1}
  const float lastSwingX = lastSwingFoot.translation.x();

  // change of the swing foot based on the planned walk
  const float plannedChange = (swingFoot.translation.x() - (lastSwingFoot.translation.x() - swingAdjustment));

  const float maxAdjustmentAfterStop = swingAdjustment - plannedChange;
  const bool hadAdjustmentLastFrame = swingAdjustment != 0.f;

  // deltaReduced = delta{hat{w_t-1}}
  const Rangef adjustmentClip(-useMaxVelX - useRemoveSpeedX + std::min(-plannedChange, 0.f), useMaxVelX + useRemoveSpeedX + std::max(-plannedChange, 0.f));
  float deltaReduced = -adjustmentClip.limit(swingAdjustment);

  // 2. low pass filter if feet are already adjusted, to reduce damage from noise
  if(std::abs(swingAdjustment + deltaReduced) > 0.1f)
    lowPassFilteredComX = walkStepParams.comLowPassRatio * lowPassFilteredComX + (1.f - walkStepParams.comLowPassRatio) * comX;
  else
    lowPassFilteredComX = comX;

  if(!groundContact)
    lowPassFilteredComX = 0.f;

  // currentPlannedSwingX = hat{w_t}
  const float currentPlannedSwingX = lastSwingX + plannedChange + deltaReduced;

  // Current feet area
  // For InWalkKicks, the swing foot must contain the CoM at all times to ensure stability
  const Rangef toleranceSpecificationSupport = Rangef((footOffset.min + footOffset.max) * desiredFootArea.min - footOffset.min, (footOffset.min + footOffset.max) * desiredFootArea.max - footOffset.min);
  const Rangef toleranceSpecificationSwing = walkStepParams.useFullSwingSole ? toleranceSpecificationSupport : Rangef(-footOffset.min, footOffset.max);

  float toleranceLowerLimit, toleranceUpperLimit, supportOffset = 0.f;

  // toleranceLowerLimit = Delta_min; toleranceUpperLimit = Delta_max;
  toleranceLowerLimit = std::min(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecificationSupport.min;
  toleranceUpperLimit = std::max(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecificationSupport.max;
  if(!walkStepParams.useFullSwingSole)
  {
    if(currentPlannedSwingX + toleranceSpecificationSwing.min > toleranceLowerLimit)
    {
      const float oldLimit = toleranceLowerLimit;
      toleranceLowerLimit = currentPlannedSwingX + toleranceSpecificationSwing.min;
      supportOffset = toleranceLowerLimit - oldLimit; // should be a positive sign
    }
    if(currentPlannedSwingX + toleranceSpecificationSwing.max < toleranceUpperLimit)
    {
      const float oldLimit = toleranceUpperLimit;
      toleranceUpperLimit = currentPlannedSwingX + toleranceSpecificationSwing.max;
      supportOffset = oldLimit - toleranceUpperLimit; // should be a negative sign
    }
  }
  // when the arms are on the back, the upper body is tilted forward.
  // if hipRot == 0, nothing changes
  const float walkHeight = std::max(supportFoot.translation.z(), swingFoot.translation.z());
  const float deltaMin = Vector2f(toleranceLowerLimit, walkHeight).rotate(-hipRot).x();
  const float deltaMax = Vector2f(toleranceUpperLimit, walkHeight).rotate(-hipRot).x();

  // 6. delta
  delta = isStepAdjustmentAllowed ? std::min(lowPassFilteredComX - deltaMin, 0.f) + std::max(lowPassFilteredComX - deltaMax, 0.f) : 0.f;

  // hat{p_max}; hat{p_min}
  const float pMin = std::min(-useMinVelX, adjustmentClip.min - plannedChange) - std::max(deltaReduced, 0.f);
  const float pMax = std::max(useMinVelX, adjustmentClip.max - plannedChange) - std::min(deltaReduced, 0.f);

  // 7. Apply adjustment
  // Apply swing foot adjustment
  swingAdjustment = (currentPlannedSwingX + std::max(pMin, std::min(pMax, delta))) - swingFoot.translation.x();

  if(std::abs(swingAdjustment) < 0.01f)
    swingAdjustment = 0.f;

  if((stepRatio >= 0.5f && highestAdjustmentX != 0.f && hadAdjustmentLastFrame && std::abs(swingAdjustment) < std::abs(maxAdjustmentAfterStop) &&
      swingAdjustment * maxAdjustmentAfterStop >= 0.f))
    adjustmentStopCounter++;
  else if(adjustmentStopCounter > 2)
    adjustmentResumeCounter++;
  if(adjustmentResumeCounter > 2)
  {
    adjustmentResumeCounter = 0;
    adjustmentStopCounter = 0;
  }
  if(adjustmentStopCounter > 2)
  {
    const Rangef swingAdjustmentStopRange(std::min(0.f, maxAdjustmentAfterStop), std::max(0.f, maxAdjustmentAfterStop));
    swingAdjustment = swingAdjustmentStopRange.limit(swingAdjustment);
    if(Approx::isZero(swingAdjustment))
      adjustmentStopCounter = 0;
  }

  swingFoot.translation.x() += swingAdjustment;

  // clip to make sure the feet pose are reachable
  swingFoot.translation.x() = clipForwardPosition.limit(swingFoot.translation.x());
  swingAdjustment = swingFoot.translation.x() - originalSwing.translation.x();

  // Apply support foot adjustment
  supportAdjustment += std::min(-pMin + useRemoveSpeedX, std::max(-pMax - useRemoveSpeedX, -swingAdjustment / 2.f - supportAdjustment));
  const Rangef supportOffsetClipRange(std::min(0.f, supportOffset), std::max(0.f, supportOffset));
  supportAdjustment -= supportOffsetClipRange.limit(supportAdjustment);
  supportFoot.translation.x() += supportAdjustment;

  supportFoot.translation.x() = clipForwardPosition.limit(supportFoot.translation.x());
  supportAdjustment = supportFoot.translation.x() - originalSupport.translation.x();

  ////////////////////////////////////////////////////////////////
  // clip to make sure we do not unintentionally touch the ball //
  ////////////////////////////////////////////////////////////////
  // but if the robot would fall, do it anyway
  /*if (delta < 25.f && !allowTouchingTheBallForBalancing) // Todo 25.f should be a parameter, and maybe change code to still clip, instead of allowing full adjustment after exceeding the 25 mm mark
  {
    const float clipAdjustment = std::max((supportFoot.translation.x() + std::max(ball.x() - clipAtBallDistanceX, 0.f)) - (swingFoot.translation.x() - swingAdjustment), 0.f) / 1.5f;
    const float clippedAdjustment = std::min(clipAdjustment - swingAdjustment, 0.f);
    swingAdjustment += clippedAdjustment;
    supportAdjustment -= clippedAdjustment / 2.f;
    swingFoot.translation.x() += clippedAdjustment;
    supportFoot.translation.x() -= clippedAdjustment / 2.f;
  }
  else
    allowTouchingTheBallForBalancing = true;*/

  // Save highest adjustments for later
  highestAdjustmentX = (swingAdjustment < 0.f ? -1.f : (swingAdjustment > 0.f ? 1.f : 0.f)) * std::max(std::abs(highestAdjustmentX), std::abs(swingAdjustment)); // save the highest value for the current step
  highestNegativeAdjustmentX = std::min(highestNegativeAdjustmentX, highestAdjustmentX);

  // Too much backward adjustment. Reduce walking speed
  if((highestNegativeAdjustmentX < -walkStepParams.unstableWalkThreshold &&  // threshold check
      frameInfo.getTimeSince(lastNormalStep) < walkStepParams.reduceWalkingSpeedTimeWindow && // only reduce if we adjusted two times backwards in the last time frame
      frameInfo.getTimeSince(lastLargeBackwardStep) < walkStepParams.reduceWalkingSpeedTimeWindow) ||
     (highestAdjustmentX > walkStepParams.unstableWalkThreshold && std::abs(highestAdjustmentX) != highestNegativeAdjustmentX)) // reduce walk speed if we adjusted a lot forward
  {
    if(reduceWalkingSpeed != reduceWalkingSpeedStepAdjustmentSteps)
      ANNOTATION("WalkingEngine", "Reduced Walking Speed!");
    reduceWalkingSpeed = reduceWalkingSpeedStepAdjustmentSteps;
  }

  // Apply accelaration control
  // When changing direction, the feet must first slow down instead of changing their direction within one frame
  // This also allows for higher adjustment speeds, as the legs will slowly speed up and therefore no jumps occure

  float movementChangeLeft = left.translation.x() - lastLeft.translation.x();
  float movementChangeRight = right.translation.x() - lastRight.translation.x();

  const float movementAccLeft = movementChangeLeft - lastMovementChangeLeft;
  const float movementAccRight = movementChangeRight - lastMovementChangeRight;

  const float constantAccSwing = swingFootHasGroundContact && stepRatio > 0.2f ? 100.f : walkStepParams.swingAcc * motionCycleTime;
  const float constantAccSupport = swingFootHasGroundContact && stepRatio > 0.2f ? 100.f : walkStepParams.supportAcc * motionCycleTime;

  const Rangef swingAcc(-constantAccSwing, constantAccSwing);
  const Rangef supportAcc(-constantAccSupport, constantAccSupport);

  const Rangef& useLeftAcc = isLeftPhase ? swingAcc : supportAcc;
  const Rangef& useRightAcc = !isLeftPhase ? swingAcc : supportAcc;

  const float leftAccAdjusted = clipForwardPosition.limit(useLeftAcc.limit(movementAccLeft) + lastMovementChangeLeft + lastLeft.translation.x()) - left.translation.x();
  const float rightAccAdjusted = clipForwardPosition.limit(useRightAcc.limit(movementAccRight) + lastMovementChangeRight + lastRight.translation.x()) - right.translation.x();

  if(std::abs(leftAccAdjusted) > 0.001f && highestAdjustmentX != 0.f)
  {
    left.translation.x() += leftAccAdjusted;
    lastLeftAdjustmentX += leftAccAdjusted;
    movementChangeLeft += leftAccAdjusted;
  }
  if(std::abs(rightAccAdjusted) > 0.001f && highestAdjustmentX != 0.f)
  {
    right.translation.x() += rightAccAdjusted;
    lastRightAdjustmentX += rightAccAdjusted;
    movementChangeRight += rightAccAdjusted;
  }

  lastMovementChangeLeft = movementChangeLeft;
  lastMovementChangeRight = movementChangeRight;
  lastLeft = left;
  lastRight = right;

  ///////////////////
  // Debug Drawings //
  ///////////////////
  COMPLEX_DRAWING3D("module:WalkStepAdjustment:balance")
  {
    Pose3f leftDrawing = left;
    Pose3f rightDrawing = right;
    Pose3f originLeftDrawing = isLeftPhase ? originalSwing : originalSupport;
    Pose3f originRightDrawing = !isLeftPhase ? originalSwing : originalSupport;
    const Vector2f leftShift = Vector2f(left.translation.x(), left.translation.z()).rotate(-hipRot);
    const Vector2f rightShift = Vector2f(right.translation.x(), right.translation.z()).rotate(-hipRot);
    const Vector2f leftOriginShift = Vector2f(originLeftDrawing.translation.x(), originLeftDrawing.translation.z()).rotate(-hipRot);
    const Vector2f rightOriginShift = Vector2f(originRightDrawing.translation.x(), originRightDrawing.translation.z()).rotate(-hipRot);
    leftDrawing.translation.x() = leftShift.x();
    leftDrawing.translation.z() = leftShift.y();
    rightDrawing.translation.x() = rightShift.x();
    rightDrawing.translation.z() = rightShift.y();
    originLeftDrawing.translation.x() = leftOriginShift.x();
    originLeftDrawing.translation.z() = leftOriginShift.y();
    originRightDrawing.translation.x() = rightOriginShift.x();
    originRightDrawing.translation.z() = rightOriginShift.y();

    // Debug Drawings
    const Pose3f maxLeft = (leftDrawing + Vector3f(toleranceSpecificationSupport.max, 0.f, 0.f));
    const Pose3f minLeft = (leftDrawing + Vector3f(toleranceSpecificationSupport.min, 0.f, 0.f));
    const Pose3f maxRight = (rightDrawing + Vector3f(toleranceSpecificationSupport.max, 0.f, 0.f));
    const Pose3f minRight = (rightDrawing + Vector3f(toleranceSpecificationSupport.min, 0.f, 0.f));

    const Pose3f& swingPos = isLeftPhase ? leftDrawing : rightDrawing;
    const Pose3f& supportPos = !isLeftPhase ? leftDrawing : rightDrawing;
    const Pose3f maxSwingPose = (swingPos + Vector3f(toleranceSpecificationSwing.max, 0.f, 0.f));
    const Pose3f minSwingPose = (swingPos + Vector3f(toleranceSpecificationSwing.min, 0.f, 0.f));

    LINE3D("module:WalkStepAdjustment:balance", maxLeft.translation.x(), maxLeft.translation.y() - 30.f, maxLeft.translation.z(), maxLeft.translation.x(), maxLeft.translation.y() + 30.f, maxLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minLeft.translation.x(), minLeft.translation.y() - 30.f, minLeft.translation.z(), minLeft.translation.x(), minLeft.translation.y() + 30.f, minLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxRight.translation.x(), maxRight.translation.y() - 30.f, maxRight.translation.z(), maxRight.translation.x(), maxRight.translation.y() + 30.f, maxRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minRight.translation.x(), minRight.translation.y() - 30.f, minRight.translation.z(), minRight.translation.x(), minRight.translation.y() + 30.f, minRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxSwingPose.translation.x(), maxSwingPose.translation.y() - 30.f, maxSwingPose.translation.z(), maxSwingPose.translation.x(), maxSwingPose.translation.y() + 30.f, maxSwingPose.translation.z(), 3, ColorRGBA::cyan);
    LINE3D("module:WalkStepAdjustment:balance", minSwingPose.translation.x(), minSwingPose.translation.y() - 30.f, minSwingPose.translation.z(), minSwingPose.translation.x(), minSwingPose.translation.y() + 30.f, minSwingPose.translation.z(), 3, ColorRGBA::cyan);
    LINE3D("module:WalkStepAdjustment:balance", toleranceUpperLimit, swingPos.translation.y() - 30.f, supportPos.translation.z(), toleranceUpperLimit, swingPos.translation.y() + 30.f, supportPos.translation.z(), 3, ColorRGBA::green);
    LINE3D("module:WalkStepAdjustment:balance", toleranceLowerLimit, swingPos.translation.y() - 30.f, supportPos.translation.z(), toleranceLowerLimit, swingPos.translation.y() + 30.f, supportPos.translation.z(), 3, ColorRGBA::green);

    LINE3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), originLeftDrawing.translation.x(), originLeftDrawing.translation.y(), originLeftDrawing.translation.z(), 2, ColorRGBA::orange);
    LINE3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), originRightDrawing.translation.x(), originRightDrawing.translation.y(), originRightDrawing.translation.z(), 2, ColorRGBA::orange);
    CROSS3D("module:WalkStepAdjustment:balance", leftDrawing.translation.x(), leftDrawing.translation.y(), leftDrawing.translation.z(), 3, 3, ColorRGBA::black);
    CROSS3D("module:WalkStepAdjustment:balance", rightDrawing.translation.x(), rightDrawing.translation.y(), rightDrawing.translation.z(), 3, 3, ColorRGBA::black);
  }

  PLOT("module:WalkStepAdjustment:Data:leftAdjustment", lastLeftAdjustmentX);
  PLOT("module:WalkStepAdjustment:Data:rightAdjustment", lastRightAdjustmentX);
}

void WalkStepAdjustment::init(const WalkStepAdjustment& walkData, const bool reset, const FrameInfo& frameInfo,
                              const float unstableWalkThreshold)
{
  swingRotXFilter.clear();
  swingRotYFilter.clear();
  if(reset)
  {
    delta = 0.f;
    return;
  }

  // For balancing
  lastLeft = walkData.lastLeft;
  lastRight = walkData.lastRight;
  lowPassFilteredComX = walkData.lowPassFilteredComX;
  previousHighestAdjustmentX = walkData.highestAdjustmentX;
  reduceWalkingSpeed = walkData.reduceWalkingSpeed;

  reduceWalkingSpeed--;
  delta = walkData.delta;

  // For walk speed reduction
  lastLargeBackwardStep = walkData.lastLargeBackwardStep;
  lastNormalStep = walkData.lastNormalStep;

  if(previousHighestAdjustmentX < -unstableWalkThreshold)
    lastLargeBackwardStep = frameInfo.time;
  if(previousHighestAdjustmentX >= 0.f)
    lastNormalStep = frameInfo.time;

  // make sure this is still low pass filtered, when balancing (otherwise for one frame the newest value is used and not the filtered one)
  lastLeftAdjustmentX = walkData.lastLeftAdjustmentX != 0.f ? (walkData.lastLeftAdjustmentX > 0.f ? 0.001f : -0.001f) : 0.f;
  lastRightAdjustmentX = walkData.lastRightAdjustmentX != 0.f ? (walkData.lastRightAdjustmentX > 0.f ? 0.001f : -0.001f) : 0.f;
}

void WalkStepAdjustment::modifySwingFootRotation(Angle& swingRotationY, const Angle oldSwingRotationY,
                                                 Angle& swingRotationX, const Angle oldSwingRotationX, const Angle torsoRotationX,
                                                 const Angle torsoRotationY, const float stepRatio,
                                                 const Angle hipRotation, const bool isLeftPhase,
                                                 const SoleRotationParameter& params, const Pose3f& measuredSwingSole,
                                                 const float plannedSideStepChange, Angle& kneeBalance, const Angle oldKneeBalance, const Angle yGyro)
{
  RingBuffer<Angle, 3>& swingRoll = isLeftPhase ? lastLeftRollRequest : lastRightRollRequest;
  RingBuffer<Angle, 3>& swingPitch = isLeftPhase ? lastLeftPitchRequest : lastRightPitchRequest;

  // 0.15 -> about 3 frames, 0.35 -> about 9 frames
  const float timeErrorFactor = Rangef::ZeroOneRange().limit((stepRatio - params.measuredErrorTimeScaling.min) / (params.measuredErrorTimeScaling.max - params.measuredErrorTimeScaling.min));
  const Angle rollExecuteError = !swingRoll.full() ? 0 : (swingRoll[swingRoll.size() - 1] - measuredSwingSole.rotation.getXAngle()) * timeErrorFactor;
  const Angle pitchExecuteError = !swingPitch.full() ? 0 : (swingPitch[swingPitch.size() - 1] - measuredSwingSole.rotation.getYAngle()) * timeErrorFactor;

  const Rangea rollRange(isLeftPhase ? 0_deg : -params.maxRollAdjustment, isLeftPhase ? params.maxRollAdjustment : 0_deg);
  const Angle rollTarget = rollRange.limit(-torsoRotationX);
  Angle pitchTarget = -(torsoRotationY + hipRotation);

  const Rangea limitAnkleSpeedPitch(std::min(0.f, oldSwingRotationY - swingRotationY) - params.soleCompensationSpeed.y() * motionCycleTime, std::max(0.f, oldSwingRotationY - swingRotationY) + params.soleCompensationSpeed.y() * motionCycleTime); // allowed angular speed
  const Rangea limitAnkleSpeedRoll(std::min(0.f, oldSwingRotationX - swingRotationX) - params.soleCompensationSpeed.x() * motionCycleTime, std::max(0.f, oldSwingRotationX - swingRotationY) + params.soleCompensationSpeed.x() * motionCycleTime); // allowed angular speed
  const Rangea limitKneeBalance(-params.soleCompensationSpeed.y() * motionCycleTime, params.soleCompensationSpeed.y() * motionCycleTime);

  const float timeFactor = Rangef::ZeroOneRange().limit((stepRatio - params.timeScaling.min) / (params.timeScaling.max - params.timeScaling.min));
  const float timeFactorRoll = Rangef::ZeroOneRange().limit((stepRatio - params.timeScalingRoll.min) / (params.timeScalingRoll.max - params.timeScalingRoll.min));
  float torsoFactor = (1.f - timeFactor) + timeFactor * params.reductionTimeFactor;
  float torsoFactorRoll = (1.f - timeFactorRoll) + timeFactorRoll * params.reductionTimeFactor;

  // So far from testing, a correction helps to prevent too much forward swing after a support foot switch
  // But it should be as minimal as possible, but still exist for backwalking. Otherwise the robot will tilt more and more backward
  // TODO might need to be even less
  if(pitchTarget > 0.f)
    torsoFactor = std::min(torsoFactor, params.soleCompensationBackwardReduction);

  const float reverseTimeFactor = 1.f - timeFactor;
  ASSERT(params.removeYCompensationAtStart <= 0);
  pitchTarget -= Rangea(reverseTimeFactor * params.removeYCompensationAtStart, 0).limit(pitchTarget);

  const float sideFactor = mapToRange(plannedSideStepChange, params.sideSizeXRotationScaling.min, params.sideSizeXRotationScaling.max, 0.f, 1.f);

  const Angle clippedCurrent = std::max(swingRotYFilter.currentValue, 0.f);
  const Angle offsetDiffAngle = mapToRange(timeErrorFactor, 0.f, 1.f, static_cast<float>(params.tiltErrorDiffOffset.max), static_cast<float>(params.tiltErrorDiffOffset.min));
  const float tiltErrorDiffFactor = params.tiltErrorDiffScaling - mapToRange(Angle(pitchExecuteError + clippedCurrent), -params.tiltErrorDiffOffset.max, -params.tiltErrorDiffOffset.min, 0_deg, Angle(params.tiltErrorDiffScaling));

  const Angle usePitchDiffError = std::min(0.f, (pitchExecuteError + offsetDiffAngle + clippedCurrent - std::min(0_deg, torsoRotationY)) - offsetDiffAngle * tiltErrorDiffFactor);
  const Angle targetSolePitch = torsoFactor * pitchTarget + pitchExecuteError * params.measuredErrorFactor + usePitchDiffError * params.tiltErrorDiffScaling * timeFactor;
  const Angle targetSoleRoll = (torsoFactorRoll * rollTarget + rollExecuteError * params.measuredErrorFactor) * sideFactor;

  const float gyroScaling = mapToRange(yGyro, params.gyroScaling.min, params.gyroScaling.max, 0_deg, Angle(1.f));
  if(kneeBalanceActive || stepRatio < mapToRange(1.f - gyroScaling, 0.f, 1.f, params.maxStepRatioToStart.min, params.maxStepRatioToStart.max))
  {
    const float useDeltaRangeValue = mapToRange(1.f - gyroScaling, 0.f, 1.f, params.deltaRange.min, params.deltaRange.max);
    const float deltaScaling = Rangef::ZeroOneRange().limit(useDeltaRangeValue >= 0 ? 0.f : delta / useDeltaRangeValue);
    const float comScaling = mapToRange(lowPassFilteredComX, params.comXRange.min, params.comXRange.max, 1.f, 0.f);

    const float torsoRangeValue = mapToRange(gyroScaling, 0.f, 1.f, float(params.torsoRange.min), float(params.torsoRange.max));
    const float torsoTiltScaling = Rangef::ZeroOneRange().limit(torsoRangeValue <= 0 ? 0.f : -(torsoRotationY + hipRotation) / torsoRangeValue);
    const float gyroScaling = params.scalingClipRange.limit(yGyro / params.minGyro);

    const float sumScaling = std::max(deltaScaling, comScaling) + torsoTiltScaling + gyroScaling;

    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:delta", deltaScaling);
    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:torso", torsoTiltScaling);
    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:gyro", gyroScaling);
    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:sum", sumScaling);

    kneeBalanceActive |= params.minMaxSumScaling.min <= sumScaling &&
                         (deltaScaling > 0.f || comScaling > 0.f) && // All values must be > 0
                         torsoTiltScaling > 0.f &&
                         gyroScaling > 0.f;

    if(kneeBalanceActive)
      kneeBalanceFactor = mapToRange(sumScaling, params.minMaxSumScaling.min, params.minMaxSumScaling.max, 0.f, 1.f);
  }

  {
    swingRotYFilter.update(swingRotationY + limitAnkleSpeedPitch.limit((1.f - kneeBalanceFactor) * targetSolePitch - swingRotationY));
    swingRotationY = swingRotYFilter.currentValue;
  }

  {
    const float otherTimeFactor = stepRatio < 0.65f ? 0.9f : (1.f - stepRatio);
    const Angle targetSolePitch = kneeBalanceFactor * std::max(otherTimeFactor * (pitchTarget + pitchExecuteError * params.measuredErrorFactor), 0.f);
    if(targetSolePitch > 0.f)
    {
      const float torsoFactor = Rangef::ZeroOneRange().limit(sqr((torsoRotationY + hipRotation) / -5_deg));
      const Rangea limitAnkleSpeedPitch(std::min(0.f, oldKneeBalance - kneeBalance) - 40_deg * motionCycleTime * (1.f + 3.f * torsoFactor), std::max(0.f, oldKneeBalance - kneeBalance) + params.soleCompensationSpeed.y() * motionCycleTime * 3.f); // allowed angular speed

      kneeBalanceFilter.update(kneeBalance + limitAnkleSpeedPitch.limit(targetSolePitch - kneeBalance));
      kneeBalance = kneeBalanceFilter.currentValue;
    }
    else
    {
      kneeBalanceActive = false;
      kneeBalanceFilter.update(0_deg);
    }
  }

  PLOT("module:WalkingEngine:Data:supportSoleRotationCompensationY", swingRotationY.toDegrees());
  PLOT("module:WalkingEngine:Data:supportSoleRotationCompensationX", swingRotationX.toDegrees());
  swingRotXFilter.update(swingRotationX + limitAnkleSpeedRoll.limit(targetSoleRoll - swingRotationX));
  swingRotationX = swingRotXFilter.currentValue;
  lastLeftRollRequest.push_front(0);
  lastLeftPitchRequest.push_front(hipRotation);

  lastRightRollRequest.push_front(0);
  lastRightPitchRequest.push_front(hipRotation);
}
