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
  kneeHipBalanceCounter = 0.f;
  hipBalanceIsSafeBackward = 0.f;
  hipBalanceIsSafeForward = 0.f;
  backwardsWalkingRotationCompensationFactor = 0.f;
  forwardsWalkingRotationCompensationFactor = 1.f;
  swingFootXTranslationChange = 0;
  lastLargeBackwardStep = 0;
  lastNormalStep = 0;
  reduceWalkingSpeed = 0;

  for(std::size_t i = 0; i < kneeBalanceFilter.buffer.capacity(); i++)
    kneeBalanceFilter.update(0.01_deg * (i + 1)); // set up buffer to ensure it can move knee as fast as possible
}

void WalkStepAdjustment::addBalance(Pose3f& left, Pose3f& right, const float stepTime, const Vector2f& com, const FootOffset& footOffset,
                                    const Rangef& clipForwardPosition, const bool isLeftPhase, const FootSupport& footSupport,
                                    const FsrData& fsrData, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                                    const int reduceWalkingSpeedStepAdjustmentSteps, const Vector2f& ball, const float clipAtBallDistanceX,
                                    const bool groundContact, const WalkStepAdjustmentParams& walkStepParams)
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
  // const float stepTimeClipped = Rangef::ZeroOneRange().limit(stepTime);
  const float comX = com.x();
  float useMaxVelX = walkStepParams.maxVelX * Constants::motionCycleTime;
  float useMinVelX = walkStepParams.minVelX * Constants::motionCycleTime;
  float useRemoveSpeedX = walkStepParams.removeSpeedX * Constants::motionCycleTime;
  const bool wrongSupportFoot = footSupport.support < 0.f != isLeftPhase;
  const bool swingFootHasGroundContact = (isLeftPhase && fsrData.legInfo[Legs::left].hasPressure == frameInfo.time) || (!isLeftPhase && fsrData.legInfo[Legs::right].hasPressure == frameInfo.time);
  if(!groundContact)
  {
    useMaxVelX = 0.f;
    useMinVelX = 0.f;
    useRemoveSpeedX = 0.f;
  }
  else if(swingFootHasGroundContact && !wrongSupportFoot)
  {
    useMaxVelX = stepTime < 0.2f ? useMaxVelX : 0.f;
    useMinVelX = stepTime < 0.2f ? useMinVelX : 0.f;
    useRemoveSpeedX = stepTime < 0.2f ? useRemoveSpeedX : 0.f;
  }
  else if(stepTime > 1.0f)
  {
    const float ratio = mapToRange(stepTime, 1.0f, 1.35f, 1.f, 0.f);
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

  if(stepTime >= 0.5f)
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
  const Rangef toleranceSpecificationSupport = Rangef((footOffset.backward + footOffset.forward) * walkStepParams.desiredFootArea.min - footOffset.backward, (footOffset.backward + footOffset.forward) * walkStepParams.desiredFootArea.max - footOffset.backward);
  //const Rangef toleranceSpecificationSwing = isInWalkKick ? toleranceSpecificationSupport : Rangef(-footOffset.backward, footOffset.forward);
  const Rangef toleranceSpecificationSwing = Rangef(-footOffset.backward, footOffset.forward);

  // toleranceLowerLimit = Delta_min; toleranceUpperLimit = Delta_max;
  float toleranceLowerLimit = std::min(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecificationSupport.min;
  float toleranceUpperLimit = std::max(currentPlannedSwingX, supportFoot.translation.x() - (swingAdjustment + deltaReduced) / 2.f) + toleranceSpecificationSupport.max;
  // toleranceLowerLimit = Delta_min; toleranceUpperLimit = Delta_max;
  float supportOffset = 0.f;
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
  // when the arms are on the back, the upper body is tilted forward.
  // if hipRot == 0, nothing changes
  const float walkHeight = std::max(supportFoot.translation.z(), swingFoot.translation.z());
  const float deltaMin = Vector2f(toleranceLowerLimit, walkHeight).rotate(-hipRot).x();
  const float deltaMax = Vector2f(toleranceUpperLimit, walkHeight).rotate(-hipRot).x();

  // 6. delta
  delta = isStepAdjustmentAllowed ? std::min(lowPassFilteredComX - deltaMin, 0.f) + std::max(lowPassFilteredComX - deltaMax, 0.f) : 0.f;

  // hat{p_max}; hat{p_min}
  const float pMin = std::min(-useMinVelX, adjustmentClip.min - plannedChange) -  std::max(deltaReduced, 0.f);
  const float pMax = std::max(useMinVelX, adjustmentClip.max - plannedChange) -  std::min(deltaReduced, 0.f);

  // 7. Apply adjustment
  // Apply swing foot adjustment
  swingAdjustment = (currentPlannedSwingX + std::max(pMin, std::min(pMax, delta))) - swingFoot.translation.x();

  if(std::abs(swingAdjustment) < 0.1f)
    swingAdjustment = 0.f;

  if((stepTime >= 0.5f && highestAdjustmentX != 0.f && hadAdjustmentLastFrame && std::abs(swingAdjustment) < std::abs(maxAdjustmentAfterStop) &&
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
  if(delta < 25.f && !allowTouchingTheBallForBalancing) // Todo 25.f should be a parameter, and maybe change code to still clip, instead of allowing full adjustment after exceeding the 25 mm mark
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

  ///////////////////////////////////////////
  // Additional balancing over engineering //
  ///////////////////////////////////////////

  // 8. shall the hip pitch be used to balance?
  Rangef offsetsHipBalance = Rangef((footOffset.backward + footOffset.forward) * (walkStepParams.desiredFootArea.min + walkStepParams.hipBalanceBackwardFootArea) - footOffset.backward, (footOffset.backward + footOffset.forward) * (walkStepParams.desiredFootArea.max - walkStepParams.hipBalanceBackwardFootArea) - footOffset.backward);
  const float toleranceLowerLimitSmall = std::min(currentPlannedSwingX - deltaReduced, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f) + offsetsHipBalance.min;
  const float toleranceUpperLimitSmall = std::max(currentPlannedSwingX - deltaReduced, supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f) + offsetsHipBalance.max;

  float smallMax = Vector2f(toleranceLowerLimitSmall, walkHeight).rotate(-hipRot).x();
  float smallMin = Vector2f(toleranceUpperLimitSmall, walkHeight).rotate(-hipRot).x();
  hipBalanceIsSafeBackward = 1.f - Rangef::ZeroOneRange().limit((lowPassFilteredComX - smallMax) / (deltaMax - smallMax));
  hipBalanceIsSafeForward = Rangef::ZeroOneRange().limit((lowPassFilteredComX - deltaMin) / (smallMin - deltaMin));//* Rangef::ZeroOneRange().limit((ball.x() - clipAtBallDistanceX) / 20.f); // TODO Parameter

  ///////////////////////////////////////////////////
  // Really worn out robots need a earlier trigger //
  ///////////////////////////////////////////////////

  // 9. Calculate toe and heel points of individuell feet and both together
  float heelFeet[Legs::numOfLegs];
  float toeFeet[Legs::numOfLegs];

  // Calculate toe and heel points for both feet
  heelFeet[Legs::left] = currentPlannedSwingX - deltaReduced - footOffset.backward;
  heelFeet[Legs::right] = supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f - footOffset.backward;
  toeFeet[Legs::left] = currentPlannedSwingX - deltaReduced + footOffset.forward;
  toeFeet[Legs::right] = supportFoot.translation.x() + (swingAdjustment + deltaReduced) / 2.f + footOffset.forward;
  // And assign swing and support to correct foot
  // The correct assignment currently does not matter
  if(!isLeftPhase)
  {
    const float heelLeft = heelFeet[Legs::left];
    heelFeet[Legs::left] = heelFeet[Legs::right];
    heelFeet[Legs::right] = heelLeft;

    const float toeLeft = toeFeet[Legs::left];
    toeFeet[Legs::left] = toeFeet[Legs::right];
    toeFeet[Legs::right] = toeLeft;
  }
  const float heel = std::min(heelFeet[Legs::left], heelFeet[Legs::right]);
  const float toe = std::max(toeFeet[Legs::left], toeFeet[Legs::right]);

  // Dynamic % threshold based on joint play. The higher the play, the less further back the com must be, to start the joint play gyro balancing
  // With this balancing lever a really bad robot should be able to walk as fast as a good robot
  const float magicBalancingParameterBack = 0.20f;
  const float magicBalancingParameterFront = 0.25f;
  if(highestNegativeAdjustmentX < walkStepParams.unstableBackWalkThreshold ||
     (toe - heel) * magicBalancingParameterBack + heel > comX)
  {
    kneeHipBalanceCounter = 4;
    isForwardBalance = true;
    forwardBalanceWasActive = false;
    balanceComIsForward = 1.f;
  }
  else if(toe - (toe - heel) * magicBalancingParameterFront < comX)
  {
    kneeHipBalanceCounter = 4;
    backwardBalanceWasActive = false;
    isBackwardBalance = true;
  }
  if(swingAdjustment > 0.f)  // I have no idea why I (Philip) added this back in Dec 2021. This sounds like overbalancing
  {
    kneeHipBalanceCounter = 4;
    isForwardBalance = true;
    forwardBalanceWasActive = false;
    balanceComIsForward = Rangef::ZeroOneRange().limit(balanceComIsForward + 0.25f); // Todo should be a parameter
  }

  if(isLeftPhase)
    swingFootXTranslationChange = left.translation.x() - lastLeft.translation.x();
  else
    swingFootXTranslationChange = right.translation.x() - lastRight.translation.x();

  // Too much backward adjustment. Reduce walking speed
  if((highestNegativeAdjustmentX < -walkStepParams.unstableWalkThreshold && // threshold check
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

    const Pose3f maxLeft2 = (leftDrawing + Vector3f(offsetsHipBalance.max, 0.f, 0.f));
    const Pose3f minLeft2 = (leftDrawing + Vector3f(offsetsHipBalance.min, 0.f, 0.f));
    const Pose3f maxRight2 = (rightDrawing + Vector3f(offsetsHipBalance.max, 0.f, 0.f));
    const Pose3f minRight2 = (rightDrawing + Vector3f(offsetsHipBalance.min, 0.f, 0.f));
    LINE3D("module:WalkStepAdjustment:balance", maxLeft.translation.x(), maxLeft.translation.y() - 30.f, maxLeft.translation.z(), maxLeft.translation.x(), maxLeft.translation.y() + 30.f, maxLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minLeft.translation.x(), minLeft.translation.y() - 30.f, minLeft.translation.z(), minLeft.translation.x(), minLeft.translation.y() + 30.f, minLeft.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxRight.translation.x(), maxRight.translation.y() - 30.f, maxRight.translation.z(), maxRight.translation.x(), maxRight.translation.y() + 30.f, maxRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", minRight.translation.x(), minRight.translation.y() - 30.f, minRight.translation.z(), minRight.translation.x(), minRight.translation.y() + 30.f, minRight.translation.z(), 3, ColorRGBA::magenta);
    LINE3D("module:WalkStepAdjustment:balance", maxSwingPose.translation.x(), maxSwingPose.translation.y() - 30.f, maxSwingPose.translation.z(), maxSwingPose.translation.x(), maxSwingPose.translation.y() + 30.f, maxSwingPose.translation.z(), 3, ColorRGBA::cyan);
    LINE3D("module:WalkStepAdjustment:balance", minSwingPose.translation.x(), minSwingPose.translation.y() - 30.f, minSwingPose.translation.z(), minSwingPose.translation.x(), minSwingPose.translation.y() + 30.f, minSwingPose.translation.z(), 3, ColorRGBA::cyan);
    LINE3D("module:WalkStepAdjustment:balance", toleranceUpperLimit, swingPos.translation.y() - 30.f, supportPos.translation.z(), toleranceUpperLimit, swingPos.translation.y() + 30.f, supportPos.translation.z(), 3, ColorRGBA::green);
    LINE3D("module:WalkStepAdjustment:balance", toleranceLowerLimit, swingPos.translation.y() - 30.f, supportPos.translation.z(), toleranceLowerLimit, swingPos.translation.y() + 30.f, supportPos.translation.z(), 3, ColorRGBA::green);

    LINE3D("module:WalkStepAdjustment:balance", maxLeft2.translation.x(), maxLeft2.translation.y() - 30.f, maxLeft2.translation.z(), maxLeft2.translation.x(), maxLeft2.translation.y() + 30.f, maxLeft2.translation.z(), 3, ColorRGBA::blue);
    LINE3D("module:WalkStepAdjustment:balance", minLeft2.translation.x(), minLeft2.translation.y() - 30.f, minLeft2.translation.z(), minLeft2.translation.x(), minLeft2.translation.y() + 30.f, minLeft2.translation.z(), 3, ColorRGBA::blue);
    LINE3D("module:WalkStepAdjustment:balance", maxRight2.translation.x(), maxRight2.translation.y() - 30.f, maxRight2.translation.z(), maxRight2.translation.x(), maxRight2.translation.y() + 30.f, maxRight2.translation.z(), 3, ColorRGBA::blue);
    LINE3D("module:WalkStepAdjustment:balance", minRight2.translation.x(), minRight2.translation.y() - 30.f, minRight2.translation.z(), minRight2.translation.x(), minRight2.translation.y() + 30.f, minRight2.translation.z(), 3, ColorRGBA::blue);

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
    kneeHipBalanceCounter = 0;
    delta = 0.f;
    return;
  }

  // For balancing
  lastLeft = walkData.lastLeft;
  lastRight = walkData.lastRight;
  lowPassFilteredComX = walkData.lowPassFilteredComX;
  previousHighestAdjustmentX = walkData.highestAdjustmentX;
  kneeHipBalanceCounter = walkData.kneeHipBalanceCounter;
  isForwardBalance = walkData.isForwardBalance;
  isBackwardBalance = walkData.isBackwardBalance;
  reduceWalkingSpeed = walkData.reduceWalkingSpeed;
  backwardsWalkingRotationCompensationFactor = walkData.backwardsWalkingRotationCompensationFactor;
  forwardsWalkingRotationCompensationFactor = walkData.forwardsWalkingRotationCompensationFactor;
  forwardBalanceWasActive = walkData.forwardBalanceWasActive;
  backwardBalanceWasActive = walkData.backwardBalanceWasActive;
  kneeHipBalanceCounter--;

  if(kneeHipBalanceCounter <= 0)
  {
    isForwardBalance = false;
    isBackwardBalance = false;
  }

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
  backwardsWalkingRotationCompensationFactor *= 0.5f;
  forwardsWalkingRotationCompensationFactor *= 0.5f;
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

  const Rangea limitAnkleSpeedPitch(std::min(0.f, oldSwingRotationY - swingRotationY) - params.soleCompensationSpeed.y() * Constants::motionCycleTime, std::max(0.f, oldSwingRotationY - swingRotationY) + params.soleCompensationSpeed.y() * Constants::motionCycleTime); // allowed angular speed
  const Rangea limitAnkleSpeedRoll(std::min(0.f, oldSwingRotationX - swingRotationX) - params.soleCompensationSpeed.x() * Constants::motionCycleTime, std::max(0.f, oldSwingRotationX - swingRotationY) + params.soleCompensationSpeed.x() * Constants::motionCycleTime); // allowed angular speed
  const Rangea limitKneeBalance(-params.soleCompensationSpeed.y() * Constants::motionCycleTime, params.soleCompensationSpeed.y() * Constants::motionCycleTime);

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
  pitchTarget -= Rangea(reverseTimeFactor * -3_deg, 0).limit(pitchTarget);

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

    const float torsoRangeValue = mapToRange(gyroScaling, 0.f, 1.f, float(params.torsoRange.min), float(params.torsoRange.max));
    const float torsoTiltScaling = Rangef::ZeroOneRange().limit(torsoRangeValue <= 0 ? 0.f : -(torsoRotationY + hipRotation) / torsoRangeValue);
    const float gyroScaling = params.scalingClipRange.limit(yGyro / params.minGyro);

    const float sumScaling = deltaScaling + torsoTiltScaling + gyroScaling;

    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:delta", deltaScaling);
    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:torso", torsoTiltScaling);
    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:gyro", gyroScaling);
    PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:sum", sumScaling);

    kneeBalanceActive |= params.minMaxSumScaling.min <= sumScaling &&
                         deltaScaling > 0.f && // All values must be > 0
                         torsoTiltScaling > 0.f &&
                         gyroScaling > 0.f;

    if(kneeBalanceActive)
      kneeBalanceFactor = mapToRange(sumScaling, params.minMaxSumScaling.min, params.minMaxSumScaling.max, 0.f, 1.f);
  }

  {
    swingRotYFilter.update(swingRotationY + limitAnkleSpeedPitch.limit((1.f - kneeBalanceFactor)*targetSolePitch - swingRotationY));
    swingRotationY = swingRotYFilter.currentValue;
  }

  {
    const float otherTimeFactor = stepRatio < 0.65f ? 0.9f : (1.f - stepRatio);
    const Angle targetSolePitch = kneeBalanceFactor * std::max(otherTimeFactor * (pitchTarget + pitchExecuteError * params.measuredErrorFactor), 0.f);
    if(targetSolePitch > 0.f)
    {
      const float torsoFactor = Rangef::ZeroOneRange().limit(sqr((torsoRotationY + hipRotation) / -5_deg));
      const Rangea limitAnkleSpeedPitch(std::min(0.f, oldKneeBalance - kneeBalance) - 40_deg * Constants::motionCycleTime * (1.f + 3.f * torsoFactor), std::max(0.f, oldKneeBalance - kneeBalance) + params.soleCompensationSpeed.y() * Constants::motionCycleTime * 3.f); // allowed angular speed

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
