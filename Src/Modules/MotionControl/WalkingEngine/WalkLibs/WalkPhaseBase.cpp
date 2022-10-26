/**
 * @file WalkPhaseBase.cpp
 * This file declares helper functions for the WalkingEngine.
 * @author Philip Reichenberg
 */

#include "WalkPhaseBase.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"

WalkPhaseBase::WalkPhaseBase(const WalkingEngine& engine, const WalkKickStep& walkKickStep) :
  MotionPhase(MotionPhase::walk),
  engine(engine),
  walkKickStep(walkKickStep)
{
}

std::vector<Vector2f> WalkPhaseBase::getTranslationPolygon(const float maxBack, const float maxFront, const float maxSide)
{
  // Get max possible requests
  Vector2f backRight, frontLeft;
  const float tFactor = 1.f;
  const float durationValue = engine.kinematicParameters.baseWalkPeriod / 1000.f;
  const float minUseMaxSpeedBackwards = engine.stepSizeParameters.minXBackwardTranslationFastRange.max / durationValue;
  const float useMaxSpeedBackwards = minUseMaxSpeedBackwards < -maxBack ? minUseMaxSpeedBackwards : -maxBack;
  backRight.x() = tFactor * useMaxSpeedBackwards * durationValue;
  backRight.y() = tFactor * -2.f * maxSide * durationValue;
  frontLeft.x() = tFactor * maxFront * durationValue;
  frontLeft.y() = tFactor * 2.f * maxSide * durationValue;

  float forwardAtMaxSide = 0.f;
  float forwardAt100 = 0.f;
  float backwardAtMaxSide = 0.f;
  float backwardAt100 = 0.f;

  auto searchPolygonBorder = [this](const Vector2f& edgePoint, float& speedAtMaxSide, float& speedAt100, const bool isFront)
  {
    for(float i = 0.f; i < 1.f; i += 1.f / std::abs(edgePoint.y()))
    {
      // dummy parameters
      float fL = 0.f;
      float fR = 0.f;
      float sL = 0.f;
      float sR = 0.f;
      float fLH0 = 0.f;
      float fRH0 = 0.f;
      float fHL = 0.f;
      float fHR = 0.f;
      Angle turn = 0_deg;
      JointRequest jointRequest;

      float scaleForward = 1.f;
      while(true)
      {
        const Pose2f useStepTarget(turn, scaleForward * edgePoint.x(), edgePoint.y() * (1.f - i));
        const Vector2f hipOffset(0.f, isLeftPhase ? engine.theRobotDimensions.yHipOffset : -engine.theRobotDimensions.yHipOffset);
        const Vector2f forwardAndSide = (useStepTarget.translation + hipOffset).rotated(-useStepTarget.rotation * 0.5f) - 2.f * hipOffset + hipOffset.rotated(useStepTarget.rotation * 0.5f);

        forwardStep = forwardAndSide.x();
        sideStep = forwardAndSide.y();

        calcFootOffsets(1.f, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, fL, fR, 0.f, 0.f, sL, sR, fLH0, fRH0, fHL, fHR, turn, forwardStep, sideStep, 0_deg);
        Pose3f leftFoot;
        Pose3f rightFoot;
        engine.calcFeetPoses(fL, fR, sL, sR, fHL, fHR, turn, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFoot, rightFoot); // current request
        leftFoot.translation.x() += engine.stepSizeParameters.translationPolygonSafeRange.min; // in case arms are on the back, the body is shifted
        rightFoot.translation.x() += engine.stepSizeParameters.translationPolygonSafeRange.min;
        if(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions))
        {
          leftFoot.translation.x() += -engine.stepSizeParameters.translationPolygonSafeRange.min + engine.stepSizeParameters.translationPolygonSafeRange.max; // some room to the balancer
          rightFoot.translation.x() += -engine.stepSizeParameters.translationPolygonSafeRange.min + engine.stepSizeParameters.translationPolygonSafeRange.max;
          if(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions))
            break;
        }
        scaleForward -= 0.005f;
        if(scaleForward < 0.f)
        {
          scaleForward = 0.f;
          break;
        }
      }

      if(i == 0.f)
        speedAtMaxSide = scaleForward * edgePoint.x();
      if(scaleForward == 1.f && speedAt100 == 0.f)
        speedAt100 = edgePoint.y() * (1.f - i);

      if(isFront)
        PLOT("module:WalkingEngine:SpeedPolygon:front", scaleForward);
      else
        PLOT("module:WalkingEngine:SpeedPolygon:back", -scaleForward);
    }
  };

  searchPolygonBorder(frontLeft, forwardAtMaxSide, forwardAt100, true);
  searchPolygonBorder(backRight, backwardAtMaxSide, backwardAt100, false);

  std::vector<Vector2f> translationPolygon;
  translationPolygon.emplace_back(Vector2f(forwardAtMaxSide, frontLeft.y()));
  translationPolygon.emplace_back(Vector2f(frontLeft.x(), forwardAt100));
  translationPolygon.emplace_back(Vector2f(frontLeft.x(), -forwardAt100));
  translationPolygon.emplace_back(Vector2f(forwardAtMaxSide, backRight.y()));
  translationPolygon.emplace_back(Vector2f(backwardAtMaxSide, backRight.y()));
  translationPolygon.emplace_back(Vector2f(backRight.x(), backwardAt100));
  translationPolygon.emplace_back(Vector2f(backRight.x(), -backwardAt100));
  translationPolygon.emplace_back(Vector2f(backwardAtMaxSide, frontLeft.y()));
  return translationPolygon;
}

void WalkPhaseBase::constructorHelperInitVariablesLastPhase(const WalkPhaseBase& lastWalkPhaseDummy, const bool resetWalkStepAdjustment)
{
  timeWhenLastKick = lastWalkPhaseDummy.timeWhenLastKick;
  armCompensationAfterKick = lastWalkPhaseDummy.armCompensationAfterKick;
  annotationTimestamp = lastWalkPhaseDummy.annotationTimestamp;
  leftArmInterpolationStart = lastWalkPhaseDummy.leftArmInterpolationStart;
  rightArmInterpolationStart = lastWalkPhaseDummy.rightArmInterpolationStart;
  leftArmInterpolationTime = lastWalkPhaseDummy.leftArmInterpolationTime;
  rightArmInterpolationTime = lastWalkPhaseDummy.rightArmInterpolationTime;
  leftArm = lastWalkPhaseDummy.leftArm;
  rightArm = lastWalkPhaseDummy.rightArm;
  forwardL0 = lastWalkPhaseDummy.forwardL;
  forwardR0 = lastWalkPhaseDummy.forwardR;
  sideL0 = lastWalkPhaseDummy.lastPrevSideL;
  sideR0 = lastWalkPhaseDummy.lastPrevSideR;
  turnRL0 = lastWalkPhaseDummy.turnRL;
  footHL0 = lastWalkPhaseDummy.footHL;
  footHR0 = lastWalkPhaseDummy.footHR;
  forwardL0 += lastWalkPhaseDummy.walkStepAdjustment.lastLeftAdjustmentX;
  forwardR0 += lastWalkPhaseDummy.walkStepAdjustment.lastRightAdjustmentX;
  soleRotationYL = lastWalkPhaseDummy.soleRotationYL * 0.5f;
  soleRotationXL = lastWalkPhaseDummy.soleRotationXL * 0.5f;
  soleRotationYR = lastWalkPhaseDummy.soleRotationYR * 0.5f;
  soleRotationXR = lastWalkPhaseDummy.soleRotationXR * 0.5f;
  afterWalkKickPhase = lastWalkPhaseDummy.walkKickStep.currentKick != WalkKicks::none && this->walkKickStep.currentKick == WalkKicks::none;
  noFastTranslationPolygonSteps = std::max(0, lastWalkPhaseDummy.noFastTranslationPolygonSteps - 1);
  armCompensationTilt = lastWalkPhaseDummy.armCompensationTilt;
  useSlowSupportFootHeightAfterKickInterpolation = lastWalkPhaseDummy.walkKickStep.useSlowSupportFootHeightAfterKickInterpolation;
  lastWalkPhaseKneeHipBalance = weightShiftStatus == weightDidShift ? lastWalkPhaseDummy.currentWalkPhaseKneeHipBalance : 0_deg;
  const Angle useSoleRotationOffsetSpeed = engine.theFrameInfo.getTimeSince(timeWhenLastKick) > engine.speedParameters.soleRotationOffsetSpeedAfterKickTime ? this->engine.speedParameters.soleRotationOffsetSpeed.max : this->engine.speedParameters.soleRotationOffsetSpeed.min;
  lastWalkPhaseKneeHipBalance -= Rangea(-useSoleRotationOffsetSpeed * Constants::motionCycleTime, useSoleRotationOffsetSpeed * Constants::motionCycleTime).limit(lastWalkPhaseKneeHipBalance);
  gyroStateTimestamp = lastWalkPhaseDummy.gyroStateTimestamp;
  stoppingCounter = lastWalkPhaseDummy.stoppingCounter;

  walkStepAdjustment.init(lastWalkPhaseDummy.walkStepAdjustment, resetWalkStepAdjustment, this->engine.theFrameInfo, this->engine.walkStepAdjustmentParams.unstableWalkThreshold);
}

void WalkPhaseBase::constructorHelperOverrideStartPositions(const bool leftReplace, const bool rightReplace)
{
  JointRequest lastRequest = this->engine.theJointRequest;
  FOREACH_ENUM(Joints::Joint, joint)
    lastRequest.angles[joint] = lastRequest.angles[joint] == JointAngles::off || lastRequest.angles[joint] == JointAngles::ignore
                                ? this->engine.theJointAngles.angles[joint]
                                : lastRequest.angles[joint];

  constructorArmCompensation(lastRequest, armCompensationTilt, 1, 1, leftReplace, rightReplace);
}

void WalkPhaseBase::constructorWalkCase(const MotionPhase& lastPhase, Pose2f& useStepTarget, Pose2f& lastStepTarget, const bool standRequested)
{
  const auto& lastWalkPhaseDummy = static_cast<const WalkPhaseBase&>(lastPhase);
  ASSERT(lastWalkPhaseDummy.walkState != standing);

  weightShiftStatus = (lastWalkPhaseDummy.supportSwitchInfo.isFootSupportSwitch && lastWalkPhaseDummy.isLeftPhase != (engine.theFootSupport.support < 0.f)) ||
                      (!lastWalkPhaseDummy.supportSwitchInfo.isFootSupportSwitch && lastWalkPhaseDummy.isLeftPhase != (engine.theFootSupport.support > 0.f)) ? weightDidShift : weightDidNotShift;
  isLeftPhase = !lastWalkPhaseDummy.supportSwitchInfo.isFootSupportSwitch ? engine.theFootSupport.support > 0.f : engine.theFootSupport.support < 0.f;
  if(lastWalkPhaseDummy.earlySupportSwitchAllowed && lastWalkPhaseDummy.tBase < engine.supportSwitchPhaseRange.min * lastWalkPhaseDummy.stepDuration)
    useStepTarget.translation.x() = -30.f;

  lastStepTarget = lastWalkPhaseDummy.step;
  doBalanceSteps = std::max(0, lastWalkPhaseDummy.doBalanceSteps - 1);
  walkState = standRequested ? stopping : walking;

  // if robot is stopping, we need 1-2 walk phases for it
  if(walkState == stopping && lastWalkPhaseDummy.walkState == stopping)
  {
    if(isStandingPossible(lastWalkPhaseDummy.forwardL, lastWalkPhaseDummy.forwardR, lastWalkPhaseDummy.lastPrevSideL, lastWalkPhaseDummy.lastPrevSideR, lastWalkPhaseDummy.turnRL, lastWalkPhaseDummy.walkStepAdjustment.lastLeftAdjustmentX, lastWalkPhaseDummy.walkStepAdjustment.lastRightAdjustmentX))
    {
      walkState = standing;
      stepDuration = engine.stepDurationSpeedTarget(engine.getSideSpeed(0.f));
      stepHeightDuration = engine.stepDurationSpeedTarget(engine.getSideSpeed(0.f));
    }
  }

  constructorHelperInitVariablesLastPhase(lastWalkPhaseDummy, false);

  // Detection does not work when the forward long was executed
  if(lastWalkPhaseDummy.walkKickStep.currentKick == WalkKicks::forwardLong)
    for(unsigned i = 0; i < engine.footStepping.feetHeightDifferenceNumberOfSamples; i++)
      supportSwingHeightDifference.push_back(0.f);

  // Walk backwards to walk away from the obstacle
  if(lastWalkPhaseDummy.earlySupportSwitchAllowed && lastWalkPhaseDummy.tBase / lastWalkPhaseDummy.stepDuration < engine.footStepping.maxSupportSwitchPhaseRangeAfterSteppingOnOpponentFeet)
  {
    useStepTarget.translation.x() = engine.footStepping.stepSizeXAfterWalkingOnOpponentFeet; // walk backwards
    useStepTarget.translation.y() = 0.f; // pull both feet together
  }

  if(lastWalkPhaseDummy.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::measured || lastWalkPhaseDummy.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::measured)
    adjustRequestByMeasuredPosition(lastWalkPhaseDummy.isLeftPhase, lastWalkPhaseDummy.tBase, lastWalkPhaseDummy.stepDuration, lastWalkPhaseDummy.walkKickStep);
  if(walkKickStep.numOfBalanceSteps > 0)
    doBalanceSteps = walkKickStep.numOfBalanceSteps;

  // Last WalkPhaseDummy was an InWalkKick. Calculate the requests of the last support foot. This is done, because of the HipPitch balancing and to get a smooth interpolation
  if(lastWalkPhaseDummy.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::request || lastWalkPhaseDummy.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::request)
    constructorHelperOverrideStartPositions((!lastWalkPhaseDummy.isLeftPhase && isLeftPhase && lastWalkPhaseDummy.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::request) ||
                                            (lastWalkPhaseDummy.isLeftPhase && !isLeftPhase && lastWalkPhaseDummy.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::request),
                                            (!lastWalkPhaseDummy.isLeftPhase && isLeftPhase && lastWalkPhaseDummy.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::request) ||
                                            (lastWalkPhaseDummy.isLeftPhase && !isLeftPhase && lastWalkPhaseDummy.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::request));

  if(walkKickStep.currentKick == WalkKicks::none)
    constructorHighDelta();

  if(walkState != standing)
  {
    lastSideStep = lastWalkPhaseDummy.sideStep;
    prevSideL = sideL0;
    prevSideR = sideR0;
    prevTurn = lastWalkPhaseDummy.prevTurn;
  }
  if(engine.theGroundContactState.contact)    // should not be !standRequest, because robot could be stuck at a goal post
  {
    if(weightShiftStatus == weightDidNotShift && lastWalkPhaseDummy.weightShiftStatus == emergencyStand && lastWalkPhaseDummy.robotIsNotMoving >= engine.emergencyStep.emergencyNotMovingCounter)
      weightShiftStatus = emergencyStep;
    else if(weightShiftStatus == weightDidNotShift)
      weightShiftStatus = emergencyStand;
  }
  engine.theWalkStepData.updateCounter(!lastWalkPhaseDummy.supportSwitchInfo.isFootSupportSwitch);
}

void WalkPhaseBase::constructorStandCase(const bool standRequested, const MotionPhase& lastPhase, const Pose2f& useStepTarget)
{
  const auto& lastWalkPhaseDummy = static_cast<const WalkPhaseBase&>(lastPhase);
  constructorHelperInitVariablesLastPhase(lastWalkPhaseDummy, true);

  walkState = standRequested ? standing : starting;
  weightShiftStatus = weightDidShift;
  if(walkState == starting)
  {
    isLeftPhase = useStepTarget.translation.y() != 0.f // first step based on side translation
                  ? useStepTarget.translation.y() > 0
                  : (useStepTarget.rotation != 0_deg ? useStepTarget.rotation > 0_deg // else first step based rotation
                     : engine.theFootSupport.support < 0.f); // otherwise based on support foot
    sideL0 = (engine.theRobotModel.soleLeft.translation.y() - engine.theRobotDimensions.yHipOffset);
    sideR0 = (engine.theRobotModel.soleRight.translation.y() + engine.theRobotDimensions.yHipOffset);
  }
  else
  {
    sideL0 = lastWalkPhaseDummy.sideL0;
    sideR0 = lastWalkPhaseDummy.sideR0;
  }
}

void WalkPhaseBase::constructorAfterKickOrGetUpCase()
{
  afterKickWalkPhase = true;
  walkState = starting;
  weightShiftStatus = weightDidShift;
  isLeftPhase = engine.theFootSupport.support < 0.f;
  constructorHelperOverrideStartPositions(true, true);

  armCompensationAfterKick = 1.f;
  const RobotModel lastRobotModel(engine.theJointRequest, engine.theRobotDimensions, engine.theMassCalibration);

  // No detetction after get up or kicking
  for(unsigned i = 0; i < engine.footStepping.feetHeightDifferenceNumberOfSamples; i++)
    supportSwingHeightDifference.push_back(0.f);

  forwardStep = 0.f;
  sideStep = -(isLeftPhase ? lastRobotModel.soleRight.translation.y() + 50.f : lastRobotModel.soleLeft.translation.y() - 50.f);
  sideStep = Rangef(isLeftPhase ? 0.f : -1000.f, isLeftPhase ? 1000.f : 0.f).limit(sideStep);

  const Vector2f target(forwardStep, sideStep);
  Vector2f p1;
  if(!Geometry::isPointInsideConvexPolygon(engine.translationPolygonAfterKick.data(), static_cast<int>(engine.translationPolygonAfterKick.size()), target))
  {
    VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(engine.translationPolygonAfterKick, Geometry::Line(Vector2f(0.f, 0.f),
                                                           target.normalized()), p1));
    sideStep = p1.y();
  }

  turnStep = 0_deg;
  step = Pose2f(turnStep, forwardStep, sideStep);
}

void WalkPhaseBase::constructorOtherCase()
{
  leftArmInterpolationStart = 0;
  rightArmInterpolationStart = 0;
  Pose3f leftFoot;
  Pose3f rightFoot;
  engine.calcFeetPoses(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFoot, rightFoot); // current request
  JointRequest request;
  request.angles.fill(JointAngles::ignore);
  JointRequest arms;
  VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), request, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);
  setArms(leftFoot, rightFoot, request, arms);
  standFactor = -1.f;
  walkState = standing;
  weightShiftStatus = weightDidShift;
  startJointAngles = engine.theJointRequest;
  Angle jointDiff = 0_deg;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(joint < Joints::firstArmJoint)
      continue;
    if(startJointAngles.angles[joint] == JointAngles::off || startJointAngles.angles[joint] == JointAngles::ignore)
      startJointAngles.angles[joint] = engine.theJointAngles.angles[joint];
    const Angle dif = startJointAngles.angles[joint] - request.angles[joint];
    const float factor = joint >= Joints::firstArmJoint && joint < Joints::firstLeftLegJoint ? 2.f : 1.f; // arms are factored more than all other joints
    jointDiff = std::max(jointDiff, Angle(std::abs(dif * factor)));
  }
  standInterpolationDuration = std::max(500.f, jointDiff / engine.standInterpolationVelocity * 1000.f);
  armCompensationAfterKick = 1.f;
  leftArmInterpolationStart = engine.theFrameInfo.time;
  rightArmInterpolationStart = engine.theFrameInfo.time;
}

void WalkPhaseBase::constructorHelperInitWalkPhase(const Pose2f& useStepTarget, const Pose2f& lastStepTarget, const bool afterKickOrGetUp, const bool standRequested)
{
  // Duration of the walking step
  stepDuration = !useSlowSupportFootHeightAfterKickInterpolation ? engine.stepDurationSpeedTarget(0.f) : (engine.kinematicParameters.baseWalkPeriod * (1.f + engine.kinematicParameters.sidewaysWalkHeightPeriodIncreaseFactor)) / 1000.f;

  // the side speed
  const float sideSpeed = engine.getSideSpeed(std::max(std::abs(useStepTarget.translation.y()), std::abs(lastStepTarget.translation.y())));

  // the speed used to determine the duration of the swing height interpolation
  float useSpeedForHeightDuration = sideSpeed;

  // when back walking, increase the interpolation for the step height too
  if(useStepTarget.translation.x() < 0.f)
  {
    // TODO make the 30 and -10 to parameters
    // TODO find out better parameters. They work good, but they were the first try without much experimenting
    const float useSpeedForBackwalking = std::max(30.f * Rangef::ZeroOneRange().limit(-useStepTarget.translation.x() / -10.f), -useStepTarget.translation.x());
    useSpeedForHeightDuration = std::max(useSpeedForHeightDuration, useSpeedForBackwalking);
  }
  useSpeedForHeightDuration = std::max(walkKickStep.useSlowSwingFootHeightInterpolation, useSpeedForHeightDuration);

  // the swing height duration
  stepHeightDuration = engine.stepDurationSpeedTarget(useSpeedForHeightDuration);

  // normal walk case
  if(!afterKickOrGetUp && walkKickStep.currentKick == WalkKicks::none)
  {
    const Pose2f usedStepTarget = standRequested ? Pose2f() : useStepTarget;
    const Vector2f hipOffset(0.f, isLeftPhase ? engine.theRobotDimensions.yHipOffset : -engine.theRobotDimensions.yHipOffset);
    const Vector2f forwardAndSide = (usedStepTarget.translation + hipOffset).rotated(-usedStepTarget.rotation * 0.5f) - 2.f * hipOffset + hipOffset.rotated(usedStepTarget.rotation * 0.5f);

    forwardStep = forwardAndSide.x();
    sideStep = forwardAndSide.y();
    turnStep = usedStepTarget.rotation;

    step = usedStepTarget;
  }
  // in walk kick case
  else if(!afterKickOrGetUp)
  {
    noFastTranslationPolygonSteps = engine.stepSizeParameters.noFastTranslationPolygonStepsNumber;
    step = useStepTarget;
    const Vector2f hipOffset(0.f, isLeftPhase ? engine.theRobotDimensions.yHipOffset : -engine.theRobotDimensions.yHipOffset);
    for(size_t i = 0; i < walkKickStep.keyframe.size(); i++)
    {
      // Main Step
      const Pose2f& useStepTarget = this->walkKickStep.keyframe[i].stepTarget;
      const Vector2f forwardAndSide = (useStepTarget.translation + hipOffset).rotated(-useStepTarget.rotation * 0.5f) - 2.f * hipOffset + hipOffset.rotated(useStepTarget.rotation * 0.5f);
      this->walkKickStep.keyframe[i].stepTargetConverted.translation = forwardAndSide;
      this->walkKickStep.keyframe[i].stepTargetConverted.rotation = this->walkKickStep.keyframe[i].stepTarget.rotation;

      // Offset swing step
      const Pose2f& useStepTargetSwing = this->walkKickStep.keyframe[i].stepTargetSwing;
      const Vector2f forwardAndSideSwing = (useStepTargetSwing.translation + hipOffset).rotated(-useStepTargetSwing.rotation * 0.5f) - 2.f * hipOffset + hipOffset.rotated(useStepTargetSwing.rotation * 0.5f);
      this->walkKickStep.keyframe[i].stepTargetSwingConverted.translation = forwardAndSideSwing;
      this->walkKickStep.keyframe[i].stepTargetSwingConverted.rotation = this->walkKickStep.keyframe[i].stepTarget.rotation;
    }
  }
  maxFootHeight = engine.kinematicParameters.baseFootLift * (walkState == starting ? engine.speedParameters.reduceSwingHeightStartingFactor * (1.f - std::abs(sideSpeed) / engine.speedParameters.maxSpeed.translation.y()) + std::abs(sideSpeed) / engine.speedParameters.maxSpeed.translation.y() : 1.f) * walkKickStep.increaseSwingHeightFactor;
};

void WalkPhaseBase::constructorHighDelta()
{
  if(!engine.theGroundContactState.contact)
    return;
  if(std::abs(walkStepAdjustment.delta) > 0.f)
  {
    // Adjust start hipPitch and anklePitch for swing foot
    if(walkStepAdjustment.delta > 0.f)
    {
      JointAngles request = engine.theJointRequest;
      const float ratio = Rangef::ZeroOneRange().limit(walkStepAdjustment.delta / engine.highDeltaScale);
      lastWalkPhaseKneeHipBalance = 0.f;
      const Joints::Joint jointHip = isLeftPhase ? Joints::lHipPitch : Joints::rHipPitch;
      const Joints::Joint jointAnkle = isLeftPhase ? Joints::lAnklePitch : Joints::rAnklePitch;
      request.angles[jointHip] = std::min(request.angles[jointHip], Angle(request.angles[jointHip] * (1.f - ratio) + engine.theJointAngles.angles[jointHip] * ratio));
      request.angles[jointAnkle] = request.angles[jointAnkle] * (1.f - ratio) + engine.theJointAngles.angles[jointAnkle] * ratio;

      constructorArmCompensation(request, armCompensationTilt, 1, 1, isLeftPhase, !isLeftPhase);
      if(isLeftPhase)
      {
        soleRotationYL = std::min(0_deg, soleRotationYL);
        soleRotationXL = 0_deg;
      }
      else
      {
        soleRotationYR = std::min(0_deg, soleRotationYR);
        soleRotationXR = 0_deg;
      }
    }
    // Adjust start hip and ankle for support foot
    // Good idea on paper, bad idea on the playing field. To many other factors exist, which make this code worse
    /*else
    {
      JointAngles request = engine.theJointRequest;
      const float ratio = Rangef::ZeroOneRange().limit(walkStepAdjustment.delta / -engine.highDeltaScale);
      lastWalkPhaseKneeHipBalance = 0.f;
      const Joints::Joint jointKneeSwing = isLeftPhase ? Joints::lKneePitch : Joints::rKneePitch;
      request.angles[jointKneeSwing] = std::max(request.angles[jointKneeSwing] + engine.negativeDeltaKneeScaleOffset * ratio, engine.theJointAngles.angles[jointKneeSwing] + engine.negativeDeltaKneeScaleOffset * ratio);

      constructorArmCompensation(request, armCompensationTilt, 1, 1, true, true);
      if(isLeftPhase)
      {
        soleRotationYL = std::max(0_deg, soleRotationYL);
        soleRotationXL = 0_deg;
      }
      else
      {
        soleRotationYR = std::max(0_deg, soleRotationYR);
        soleRotationXR = 0_deg;
      }
    }*/
  }
}

void WalkPhaseBase::constructorWeightShiftStatus()
{
  if(weightShiftStatus == emergencyStand)
  {
    ANNOTATION("WalkingEngine", "emergencyStand");
    SystemCall::say("Stand");
    forwardStep = 0.f;
    sideStep = 0.f;
    turnStep = 0_deg;
    step = Pose2f();
  }
  else if(weightShiftStatus == emergencyStep)
  {
    ANNOTATION("WalkingEngine", "emergencyStep");
    SystemCall::say("Emergency");
    forwardStep = 0.f;
    isLeftPhase = !isLeftPhase;
    sideStep = isLeftPhase ? engine.emergencyStep.emergencyStepSize : -engine.emergencyStep.emergencyStepSize;
    turnStep = 0_deg;
    maxFootHeight *= engine.emergencyStep.emergencyStepHeightFactor;
    step = Pose2f();
  }
}

void WalkPhaseBase::detectWalkingOnOpponentFeet()
{
  const Legs::Leg leg = isLeftPhase ? Legs::right : Legs::left;
  if(walkState == standing || !engine.theFootSoleRotationCalibration.footCalibration[leg].isCalibrated ||
     supportSwingHeightDifference.size() >= engine.footStepping.feetHeightDifferenceNumberOfSamples || !engine.footStepping.useSteppingOnOpponentFootBehavior)
    return;

  // Fail-Safe, only if we know there is an obstacle "near", otherwise fill the list to skip checks in the current walking step
  if(supportSwingHeightDifference.empty())
  {
    bool obstacleNear = false;
    for(Obstacle ob : engine.theObstacleModel.obstacles)
    {
      obstacleNear |= ob.center.squaredNorm() < sqr(engine.footStepping.maxObstacleDistance);
      if(obstacleNear)
        break;
    }
    if(!obstacleNear)
    {
      for(unsigned i = 0; i < engine.footStepping.feetHeightDifferenceNumberOfSamples; i++)
        supportSwingHeightDifference.push_back(0.f);
      return;
    }
  }

  // Get soles in torso

  Pose3f tiltInTorso(RotationMatrix(Rotation::AngleAxis::unpack(Vector3f(-engine.theInertialData.angle.x() / 2.f, -engine.theInertialData.angle.y(), 0.f))));
  const Pose3f inTorsoLeft = tiltInTorso.inverse() * engine.theRobotModel.soleLeft;
  const Pose3f inTorsoRight = tiltInTorso.inverse() * engine.theRobotModel.soleRight;

  float groundPointLeft;
  float groundPointRight;

  // get highest point from the toe of the support foot and the lowest point of the swing foot
  if(isLeftPhase)
  {
    groundPointLeft = (inTorsoLeft * Vector3f(inTorsoLeft.rotation.col(2).x() >= 0.f ? engine.theFootOffset.forward : -engine.theFootOffset.backward, inTorsoLeft.rotation.col(2).y() >= 0.f ? engine.theFootOffset.leftFoot.left : -engine.theFootOffset.leftFoot.right, 0.f)).z();
    groundPointRight = (inTorsoRight * Vector3f(engine.theFootOffset.forward, inTorsoRight.rotation.col(2).y() < 0.f ? engine.theFootOffset.rightFoot.left : -engine.theFootOffset.rightFoot.right, 0.f)).z();
    supportSwingHeightDifference.push_back(groundPointLeft - groundPointRight);
  }
  else
  {
    groundPointLeft = (inTorsoLeft * Vector3f(engine.theFootOffset.forward, inTorsoLeft.rotation.col(2).y() < 0.f ? engine.theFootOffset.leftFoot.left : -engine.theFootOffset.leftFoot.right, 0.f)).z();
    groundPointRight = (inTorsoRight * Vector3f(inTorsoRight.rotation.col(2).x() >= 0.f ? engine.theFootOffset.forward : -engine.theFootOffset.backward, inTorsoRight.rotation.col(2).y() >= 0.f ? engine.theFootOffset.rightFoot.left : -engine.theFootOffset.rightFoot.right, 0.f)).z();
    supportSwingHeightDifference.push_back(groundPointRight - groundPointLeft);
  }

  // After enough samples, check some thresholds
  if(supportSwingHeightDifference.size() == engine.footStepping.feetHeightDifferenceNumberOfSamples &&  // we have all samples
     supportSwingHeightDifference[0] < engine.footStepping.minFeetHeightDifference && // first sample threshold check
     supportSwingHeightDifference[engine.footStepping.feetHeightDifferenceNumberOfSamples - 1] < engine.footStepping.minFeetHeightDifference && // last sample threshold check
     supportSwingHeightDifference[0] - supportSwingHeightDifference[engine.footStepping.feetHeightDifferenceNumberOfSamples - 1] > engine.footStepping.maxFeetHeightDifferenceVelocity * (1.f + std::max(0.f, (engine.footStepping.minFeetHeightDifference - supportSwingHeightDifference[0]) / engine.footStepping.maxFeetHeightDifferenceScaling)) &&
     walkStepAdjustment.previousHighestAdjustmentX > engine.footStepping.maxLastBackwardStepAdjustment) // if the robot is already tilting backward, then we can be sure what is actually happening
  {
    stepHeightDuration = 0.f;
    earlySupportSwitchAllowed = true;
    ANNOTATION("WalkingEngine", "Detected walking step on opponent feet");
    // SystemCall::playSound("doh.wav"); // As a direct feedback, so we know the robots knows what happend
    // TODO in theory we also need to make sure the current swing foot gets ground contact as fast as possible.
    // Often the detection is not enough because the next support switch happens after 50% of the step duration
  }
  PLOT("module:WalkingEngine:Data:feetHeightDifference", supportSwingHeightDifference.back());
}

void WalkPhaseBase::debugDrawingFeetPositions(const JointRequest& jointRequest)
{
  DEBUG_RESPONSE("module:WalkingEngine:feetPositions")
  {
    RobotModel requestedModel(jointRequest, engine.theRobotDimensions, engine.theMassCalibration);
    Vector3f& left = requestedModel.soleLeft.translation;
    Vector3f& right = requestedModel.soleRight.translation;
    PLOT("module:WalkingEngine:leftFootHightCurrent", engine.theRobotModel.soleLeft.translation.z());
    PLOT("module:WalkingEngine:leftFootHightDesired", left.z());
    PLOT("module:WalkingEngine:rightFootHightCurrent", engine.theRobotModel.soleRight.translation.z());
    PLOT("module:WalkingEngine:rightFootHightDesired", right.z());
    PLOT("module:WalkingEngine:leftFootForwardCurrent", engine.theRobotModel.soleLeft.translation.x());
    PLOT("module:WalkingEngine:leftFootForwardDesired", left.x());
    PLOT("module:WalkingEngine:rightFootForwardCurrent", engine.theRobotModel.soleRight.translation.x());
    PLOT("module:WalkingEngine:rightFootForwardDesired", right.x());
    PLOT("module:WalkingEngine:leftFootSideCurrent", engine.theRobotModel.soleLeft.translation.y());
    PLOT("module:WalkingEngine:leftFootSideDesired", left.y());
    PLOT("module:WalkingEngine:rightFootSideCurrent", engine.theRobotModel.soleRight.translation.y());
    PLOT("module:WalkingEngine:rightFootSideDesired", right.y());
  }
}

void WalkPhaseBase::calcJointsHelperFootSoleRotations(const JointRequest& jointRequest, const Angle hipRotation)
{
  // Reset feet rotation offsets back to 0
  const Angle useSoleRotationOffsetSpeed = engine.theFrameInfo.getTimeSince(timeWhenLastKick) > engine.speedParameters.soleRotationOffsetSpeedAfterKickTime ? this->engine.speedParameters.soleRotationOffsetSpeed.max : this->engine.speedParameters.soleRotationOffsetSpeed.min;
  const Rangea limitAnkleOffset(-useSoleRotationOffsetSpeed * Constants::motionCycleTime, useSoleRotationOffsetSpeed * Constants::motionCycleTime);
  const Angle soleYLOld = soleRotationYL;
  const Angle soleYROld = soleRotationYR;
  soleRotationYL -= limitAnkleOffset.limit(soleRotationYL);
  soleRotationXL -= limitAnkleOffset.limit(soleRotationXL);
  soleRotationYR -= limitAnkleOffset.limit(soleRotationYR);
  soleRotationXR -= limitAnkleOffset.limit(soleRotationXR);

  // Add sole rotation compensation
  if(walkState != standing && engine.theGroundContactState.contact)
  {
    RobotModel balanced(jointRequest, engine.theRobotDimensions, engine.theMassCalibration);
    walkStepAdjustment.modifySwingFootRotation(isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft,
                                               isLeftPhase ? balanced.soleRight : balanced.soleLeft,
                                               isLeftPhase ? soleRotationYL : soleRotationYR,
                                               isLeftPhase ? soleYLOld : soleYROld,
                                               engine.theInertialData.angle.y(), tBase / stepDuration,
                                               useSoleRotationOffsetSpeed,
                                               engine.theFootSoleRotationCalibration.footCalibration[Legs::left].isCalibrated && engine.theFootSoleRotationCalibration.footCalibration[Legs::right].isCalibrated,
                                               hipRotation,
                                               engine.soleRotationParameter, isLeftPhase ? balanced.soleLeft : balanced.soleRight);
  }
}

void WalkPhaseBase::calcJointsHelperInterpolateStanding(const MotionRequest& motionRequest)
{
  if(standFactor != 1.f)
    timeWhenStandHighBegan = engine.theFrameInfo.time;

  wasStandingStillOnce |= robotIsNotMoving >= engine.standHighNotMovingCounter;
  const bool highStandRequested = motionRequest.motion == MotionRequest::stand && motionRequest.standHigh && wasStandingStillOnce;
  if(standFactor < 0.f)
    standFactor = std::min(standFactor + 1000.f * Constants::motionCycleTime / standInterpolationDuration, 0.f);
  else if(highStandRequested)
  {
    const float nextFactor = std::min((std::asin(standFactor * 2.f - 1.f) + Constants::pi_2) / Constants::pi + 1000.f * Constants::motionCycleTime / engine.standHighInterpolationDuration, 1.f);
    standFactor = (std::sin(-Constants::pi_2 + Constants::pi * nextFactor) + 1.f) * 0.5f;
    standFactor = std::min(standFactor, 1.f);
  }
  else
    standFactor = std::max(standFactor - 1000.f * Constants::motionCycleTime / engine.standHighInterpolationDuration, 0.f);

  const float standFactor01 = std::max(standFactor, 0.f);
  forwardL0 = forwardR0 = 0.5f * (1.f - std::cos(standFactor01 * pi)) * engine.kinematicParameters.torsoOffset;
  footHL0 = footHR0 = -std::sin(standFactor01 * pi_2) *
                      (engine.theRobotDimensions.upperLegLength + engine.theRobotDimensions.lowerLegLength + engine.theRobotDimensions.footHeight -
                       engine.kinematicParameters.walkHipHeight - 0.0001f);
  turnRL0 = sideL0 = sideR0 = 0.f;

  forwardL = forwardL0;
  forwardR = forwardR0;
  sideL = sideL0;
  sideR = sideR0;
  footHL = footHL0;
  footHR = footHR0;
  turnRL = turnRL0;
}

Pose2f WalkPhaseBase::getOdometryOffset() const
{
  if(walkState == standing)
    return Pose2f(0_deg, 0.f, 0.f);

  return engine.theOdometryDataPreview.odometryChange;
}

float WalkPhaseBase::parabolicFootHeight(const float currentTime, const float walkHeightDuration) const
{
  float f = currentTime / walkHeightDuration;
  const float maxHeightAtRatio = std::min(engine.parabolicFootHeightParameters.maxHeightAfterTime / (walkHeightDuration * 1000.f), engine.parabolicFootHeightParameters.maxHeightAfterTimePercent);

  if(f <= maxHeightAtRatio)
    f = f / maxHeightAtRatio * 0.5f;
  else
    f = 0.5f + (f - maxHeightAtRatio) / (1.f - maxHeightAtRatio) * 0.5f;

  Rangef::ZeroOneRange().clamp(f);

  if(f < 0.25f)
    return 8.f * f * f;
  else if(f < 0.75f)
  {
    const float x = f - 0.5f;
    return 1.f - 8.f * x * x;
  }
  else
  {
    const float x = 1.f - f;
    return 8.f * x * x;
  }
}

float WalkPhaseBase::parabolicStep(float time, float period) const
{
  const float timeFraction = linearStep(time, period);
  if(timeFraction < 0.5f)
    return 2.f * timeFraction * timeFraction;
  else
    return 4.f * timeFraction - 2.f * timeFraction * timeFraction - 1.f;
}

float WalkPhaseBase::sinusMaxToZeroStep(float time, float period) const
{
  return std::sin(Constants::pi_2 * linearStep(time, period));
}

float WalkPhaseBase::linearStep(float time, float period) const
{
  return period == 0.f ? 0.f : Rangef::ZeroOneRange().limit(time / period);
}

float WalkPhaseBase::cosinusZeroToMaxStep(float time, float period) const
{
  return std::cos(-Constants::pi + Constants::pi_2 * linearStep(time, period)) + 1.f;
}

float WalkPhaseBase::swingInterpolation(float time, float period, const WalkKickStep::InterpolationType type) const
{
  switch(type)
  {
    case WalkKickStep::InterpolationType::linear:
      return linearStep(time, period);
    case WalkKickStep::InterpolationType::sinusMaxToZero:
      return sinusMaxToZeroStep(time, period);
    case WalkKickStep::InterpolationType::cosinusZeroToMax:
      return cosinusZeroToMaxStep(time, period);
    default:
      return parabolicStep(time, period);
  }
}

void WalkPhaseBase::balanceFeetPoses(Pose3f& leftFoot, Pose3f& rightFoot)
{
  RobotModel lightModel = engine.theRobotModel;
  lightModel.updateCenterOfMass(engine.lightMassCalibration);

  // Predict CoM position (diff)
  const RotationMatrix tiltInTorso(Rotation::AngleAxis::unpack(Vector3f(-engine.theInertialData.angle.x(), -engine.theInertialData.angle.y(), 0.f)));
  const RotationMatrix rot = walkStepAdjustment.predictRotation(tiltInTorso, lightModel, engine.theFootOffset, isLeftPhase, !afterKickWalkPhase);
  const Vector3f sole = rot.inverse() * (isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft).translation;
  const Vector3f comInTorso = rot * (Vector3f() << (rot.inverse() * lightModel.centerOfMass).head<2>(), sole.z()).finished();
  const Vector2f diff = comInTorso.head<2>();

  // if the feet are moving sideways, the maximal forward and backward positions must be reduced
  const float legLength = (engine.kinematicParameters.walkHipHeight - engine.theRobotDimensions.footHeight);
  const Angle sideSupport = std::atan2(sideL, legLength);
  const Angle sideSwing = std::atan2(sideR, legLength);
  const Angle useAngle = sideSupport > sideSwing ? sideSupport : sideSwing;
  const Rangef clipForward(std::min(engine.clipForward.min * (1.f - useAngle), std::min(leftFoot.translation.x(), rightFoot.translation.x())),
                           std::max(engine.clipForward.max * (1.f - useAngle), std::max(leftFoot.translation.x(), rightFoot.translation.x())));

  walkStepAdjustment.addBalance(leftFoot, rightFoot, tBase / stepDuration, diff, engine.theFootOffset, clipForward,
                                isLeftPhase, engine.theFootSupport, engine.theFsrData, engine.theFrameInfo,
                                armCompensationTilt, walkKickStep.currentKick == WalkKicks::none && walkState != standing, // should step adjustment be active?
                                engine.speedParameters.reduceWalkingSpeedStepAdjustmentSteps, ball, engine.clipAtBallDistanceX,
                                engine.theJointPlay.qualityOfRobotHardware, engine.theGroundContactState.contact,
                                engine.walkStepAdjustmentParams);

  if(engine.theFrameInfo.getTimeSince(annotationTimestamp) > 10000 && (std::abs(walkStepAdjustment.lastLeftAdjustmentX) > engine.walkStepAdjustmentParams.unstableWalkThreshold || std::abs(walkStepAdjustment.lastRightAdjustmentX) > engine.walkStepAdjustmentParams.unstableWalkThreshold))
  {
    annotationTimestamp = engine.theFrameInfo.time;
    ANNOTATION("WalkingEngine", "Step adjusted");
  }
}

void WalkPhaseBase::addGyroBalance(JointRequest& jointRequest)
{
  // Sagittal balance
  Angle balanceAdjustment = walkState == standing ? 0.f : engine.filteredGyroY *
                            (engine.filteredGyroY > 0
                             ? engine.balanceParameters.gyroForwardBalanceFactor
                             : engine.balanceParameters.gyroBackwardBalanceFactor); // adjust ankle tilt in proportion to filtered gyroY
  jointRequest.angles[isLeftPhase ? Joints::rAnklePitch : Joints::lAnklePitch] += balanceAdjustment;
  PLOT("module:WalkingEngine:Data:sagital", balanceAdjustment.toDegrees());

  // Balance with the hipPitch for one motion phase after a kick phase.
  if(walkKickStep.currentKick != WalkKicks::none || afterWalkKickPhase || engine.theFrameInfo.getTimeSince(timeWhenLastKick) < engine.speedParameters.soleRotationOffsetSpeedAfterKickTime)
  {
    const Angle balancingValue = engine.filteredGyroY *
                                 (engine.filteredGyroY > 0
                                  ? engine.balanceParameters.gyroForwardBalanceFactorHipPitch.y() * walkStepAdjustment.hipBalanceIsSafeBackward
                                  : engine.balanceParameters.gyroForwardBalanceFactorHipPitch.x()) * walkStepAdjustment.hipBalanceIsSafeForward;
    PLOT("module:WalkingEngine:Data:hipBalancing", balancingValue.toDegrees());
    jointRequest.angles[isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += balancingValue;
  }

  // Lateral balance
  if(walkState == standing && engine.theGroundContactState.contact)
  {
    balanceAdjustment = engine.filteredGyroX * engine.balanceParameters.gyroSidewaysBalanceFactor;
    jointRequest.angles[Joints::lAnkleRoll] += balanceAdjustment;
    jointRequest.angles[Joints::rAnkleRoll] += balanceAdjustment;
    PLOT("module:WalkingEngine:Data:lateral", balanceAdjustment.toDegrees());
  }

  if(walkState != standing && engine.theGroundContactState.contact)
  {
    const Angle balanceValueXRot = engine.filteredGyroX * 0.05f * walkStepAdjustment.sideBalanceFactor;
    jointRequest.angles[isLeftPhase ? Joints::rAnkleRoll : Joints::lAnkleRoll] += balanceValueXRot;
  }

  // compensate Arms
  jointRequest.angles[Joints::lHipPitch] += std::max(standFactor, 0.f) * engine.standHighTorsoPitch;
  jointRequest.angles[Joints::rHipPitch] += std::max(standFactor, 0.f) * engine.standHighTorsoPitch;

  // The robot was tilting backward or forward and is now falling in the other direction, as a result from the pendulum like swinging.
  // To stabilize the robot, the knee and hip pitch are adjusted, to prevent extra momentum resulting from the joints themself.
  if(walkStepAdjustment.kneeHipBalanceCounter > 0)
  {
    walkStepAdjustment.forwardBalanceWasActive |= engine.filteredGyroY > engine.balanceParameters.gyroBalanceKneeNegativeGyroAbort;
    if(walkStepAdjustment.isForwardBalance && engine.filteredGyroY < engine.balanceParameters.gyroBalanceKneeNegativeGyroAbort && walkStepAdjustment.forwardBalanceWasActive)
    {
      walkStepAdjustment.kneeHipBalanceCounter = 0;
      walkStepAdjustment.isForwardBalance = false;
    }

    walkStepAdjustment.backwardBalanceWasActive |= engine.filteredGyroY < engine.balanceParameters.gyroBalanceKneeNegativeGyroAbort;
    if(walkStepAdjustment.isBackwardBalance && engine.filteredGyroY > -engine.balanceParameters.gyroBalanceKneeNegativeGyroAbort && walkStepAdjustment.backwardBalanceWasActive)
    {
      walkStepAdjustment.kneeHipBalanceCounter = 0;
      walkStepAdjustment.isBackwardBalance = false;
    }
  }
  currentWalkPhaseKneeHipBalance = 0_deg;
  if((tBase > stepDuration || walkStepAdjustment.kneeHipBalanceCounter > 0) &&
     walkState != standing && engine.theGroundContactState.contact)
  {
    const Rangef balanceRange(walkStepAdjustment.isBackwardBalance ? -engine.balanceParameters.maxGyroBalanceKneeValue : 0_deg, walkStepAdjustment.isForwardBalance || tBase > stepDuration ? engine.balanceParameters.maxGyroBalanceKneeValue : 0_deg);
    float factor = 1.f;
    if(!engine.theGroundContactState.contact)
      factor *= Rangef::ZeroOneRange().limit(1.f - std::min(engine.theFrameInfo.getTimeSince(engine.theFsrData.legInfo[Legs::left].hasPressure), engine.theFrameInfo.getTimeSince(engine.theFsrData.legInfo[Legs::right].hasPressure)) / 100.f);

    const float useIsSafeFactor = (engine.filteredGyroY > 0.f ? walkStepAdjustment.hipBalanceIsSafeForward : walkStepAdjustment.hipBalanceIsSafeBackward * (tBase > stepDuration ? 1.f : walkStepAdjustment.balanceComIsForward));
    currentWalkPhaseKneeHipBalance = balanceRange.limit(engine.filteredGyroY * engine.balanceParameters.gyroBalanceKneeBalanceFactor) * factor * useIsSafeFactor;
    jointRequest.angles[isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += currentWalkPhaseKneeHipBalance;
    jointRequest.angles[isLeftPhase ? Joints::rKneePitch : Joints::lKneePitch] += currentWalkPhaseKneeHipBalance;
  }
  jointRequest.angles[!isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += lastWalkPhaseKneeHipBalance;
  jointRequest.angles[!isLeftPhase ? Joints::rKneePitch : Joints::lKneePitch] += lastWalkPhaseKneeHipBalance;
  PLOT("module:WalkingEngine:Data:kneeHipBalance", currentWalkPhaseKneeHipBalance.toDegrees());
}

bool WalkPhaseBase::isStandingPossible(float forwardL, float forwardR, float sideL, float sideR, float turnRL, float balanceAdjustmentLeft, float balanceAdjustmentRight) const
{
  // when walking normally with ground contact, one step to go into neutral is enough. when picked up, execute neutral walk phases until the legs are close together
  return ((!engine.theGroundContactState.contact &&
           std::abs(forwardL + balanceAdjustmentLeft) < engine.thresholdStopStandTransition.translation.x() &&
           std::abs(forwardR + balanceAdjustmentRight) < engine.thresholdStopStandTransition.translation.x() &&
           std::abs(sideL) < engine.thresholdStopStandTransition.translation.y() &&
           std::abs(sideR) < engine.thresholdStopStandTransition.translation.y() &&
           std::abs(turnRL) < engine.thresholdStopStandTransition.rotation)
          || (engine.theGroundContactState.contact && (!engine.blockStoppingWithStepAdjustment || (walkStepAdjustment.highestAdjustmentX == 0.f))));
}

void WalkPhaseBase::checkGyroState(const bool ignoreXGyro)
{
  if(engine.theGyroState.timestamp != gyroStateTimestamp)
  {
    gyroStateTimestamp = engine.theGyroState.timestamp;
    const bool xCheck = (std::abs(engine.theGyroState.deviation.x()) < engine.emergencyStep.emergencyMaxGyroDeviation && std::abs(engine.theGyroState.mean.x()) < engine.emergencyStep.emergencyMaxGyroMean) || ignoreXGyro;
    const bool yCheck = std::abs(engine.theGyroState.deviation.y()) < engine.emergencyStep.emergencyMaxGyroDeviation && std::abs(engine.theGyroState.mean.y()) < engine.emergencyStep.emergencyMaxGyroMean;
    const bool zCheck = std::abs(engine.theGyroState.deviation.z()) < engine.emergencyStep.emergencyMaxZGyroDeviation && std::abs(engine.theGyroState.mean.z()) < engine.emergencyStep.emergencyMaxGyroMean;
    if(xCheck && yCheck && zCheck)
      robotIsNotMoving++;
    else
      robotIsNotMoving = 0;
  }
}

void WalkPhaseBase::applyWalkKickLongKickOffset(JointRequest& jointRequest, const float time)
{
  if(walkKickStep.currentKick == WalkKicks::none)
    return;
  for(WalkKickStep::LongKickParams param : walkKickStep.longKickParams)
  {
    const Joints::Joint joint = isLeftPhase ? param.joint : Joints::mirror(param.joint);
    const float useTime = std::max(0.f, time / stepDuration < param.middleRatio ? time - stepDuration* param.minRatio : time - stepDuration * param.middleRatio);
    const float useDuration = std::max(0.f, time / stepDuration < param.middleRatio ? stepDuration * (param.middleRatio - param.minRatio) : stepDuration * (param.maxRatio - param.middleRatio));
    if(useDuration > 0.f)
    {
      const float interpolation = Rangef::ZeroOneRange().limit(std::sin((time / stepDuration < param.middleRatio ? 0.f : Constants::pi / 2.f) + Constants::pi / 2.f * std::min(1.f, useTime / useDuration)));
      float sign = 1.f;
      if((joint == Joints::lAnkleRoll
          || joint == Joints::rAnkleRoll
          || joint == Joints::lHipRoll
          || joint == Joints::rHipRoll) && !isLeftPhase)
        sign *= -1.f;
      jointRequest.angles[joint] += sign * param.offset * interpolation;
    }
  }
}

void WalkPhaseBase::adjustRequestByMeasuredPosition(const bool isLeftPhase, const float time, const float duration, const WalkKickStep& walkKickStep)
{
  if(walkKickStep.longKickParams.size() > 0)
  {
    constructorArmCompensation(engine.theJointAngles, armCompensationTilt, duration, time,
                               (isLeftPhase && walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::measured) || (!isLeftPhase && walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::measured),
                               (!isLeftPhase && walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::measured) || (isLeftPhase && walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::measured));
  }
}

void WalkPhaseBase::calculateBallPosition(const bool isLeftPhase)
{
  // TODO in theory we want to calculate it in every frame and calc the max x translation for the swing foot
  // The current use is just wrong in every way!
  if(engine.theFrameInfo.getTimeSince(engine.theMotionRequest.ballTimeWhenLastSeen) < 1000.f)
  {
    const Pose3f supportInTorso3D = engine.theTorsoMatrix * (isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -engine.theRobotDimensions.yHipOffset : engine.theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * engine.theOdometryDataPreview.inverse() * engine.theMotionRequest.odometryData;
    ball = scsCognition * engine.theMotionRequest.ballEstimate.position - Vector2f(std::max(step.translation.x(), 0.f), step.translation.y());
    if(!(ball.squaredNorm() > sqr(engine.clipAtBallDistance) || ball.x() <= 0.f))
      return;
  }
  ball = Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
}

void WalkPhaseBase::constructorArmCompensation(const JointAngles& jointAngles, const Angle theArmCompensation,
                                               const float stepDuration, const float t,
                                               const bool overrideLeftFoot, const bool overrideRightFoot)
{
  if(!overrideLeftFoot && !overrideRightFoot)
    return;

  const RobotModel model(jointAngles, engine.theRobotDimensions, engine.theMassCalibration);
  Pose3f leftSole = model.soleLeft;
  Pose3f rightSole = model.soleRight;

  Pose3f rot(Rotation::aroundY(-theArmCompensation));

  if(overrideLeftFoot)
    leftSole = rot * leftSole; // revert rotation by adding both rotations, and rotate translation of sole
  if(overrideRightFoot)
    rightSole = rot * rightSole; // revert rotation by adding both rotations, and rotate translation of sole

  turnRL0 = (leftSole.rotation.getZAngle() - rightSole.rotation.getZAngle()) * 0.5f;

  if(overrideLeftFoot)
  {
    soleRotationYL = leftSole.rotation.getYAngle();
    soleRotationXL = leftSole.rotation.getXAngle();
    forwardL0 = leftSole.translation.x() + engine.kinematicParameters.torsoOffset;
    footHL0 = engine.kinematicParameters.walkHipHeight + leftSole.translation.z() - (maxFootHeight * parabolicFootHeight(t, stepDuration));
    sideL0 = leftSole.translation.y() - this->engine.theRobotDimensions.yHipOffset;
  }
  if(overrideRightFoot)
  {
    soleRotationYR = rightSole.rotation.getYAngle();
    soleRotationXR = rightSole.rotation.getXAngle();
    forwardR0 = rightSole.translation.x() + engine.kinematicParameters.torsoOffset;
    footHR0 = engine.kinematicParameters.walkHipHeight + rightSole.translation.z() - (maxFootHeight * parabolicFootHeight(t, stepDuration));
    sideR0 = rightSole.translation.y() + this->engine.theRobotDimensions.yHipOffset;
  }
}
