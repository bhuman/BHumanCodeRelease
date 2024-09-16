/**
 * @file WalkPhaseBase.cpp
 * This file declares helper functions for the WalkingEngine.
 * @author Philip Reichenberg
 */

#include "WalkPhaseBase.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"
#include "Tools/Motion/Transformation.h"

using namespace Motion::Transformation;

WalkPhaseBase::WalkPhaseBase(WalkingEngine& engine, const WalkKickStep& walkKickStep) :
  MotionPhase(MotionPhase::walk),
  engine(engine),
  walkKickStep(walkKickStep)
{
}

std::vector<Vector2f> WalkPhaseBase::getTranslationPolygon(const float maxBack, const float maxFront, const float maxSide, const bool useSafeSpace)
{
  // Get max possible requests
  Vector2f backRight, frontLeft;
  const float durationValue = engine.kinematicParameters.baseWalkPeriod / 1000.f;
  const float minUseMaxSpeedBackwards = engine.stepSizeParameters.minXBackwardTranslationFastRange.max / durationValue;
  const float useMaxSpeedBackwards = minUseMaxSpeedBackwards < -maxBack ? minUseMaxSpeedBackwards : -maxBack;
  backRight.x() = useMaxSpeedBackwards * durationValue;
  backRight.y() = -2.f * maxSide * durationValue;
  frontLeft.x() = maxFront * durationValue;
  frontLeft.y() = 2.f * maxSide * durationValue;

  float forwardAtMaxSide = 0.f;
  float forwardAt100 = 0.f;
  float backwardAtMaxSide = 0.f;
  float backwardAt100 = 0.f;

  auto searchPolygonBorder = [&](const Vector2f& edgePoint, float& speedAtMaxSide, float& speedAt100, const bool isFront, const bool useSafeSpace)
  {
    for(float i = 0.f; i < 1.f; i += 1.f / std::abs(edgePoint.y()))
    {
      // dummy parameters
      float fL0 = 0.f;
      float fR0 = 0.f;
      float fL = 0.f;
      float fR = 0.f;
      float sL0 = 0.f;
      float sR0 = 0.f;
      float sL = 0.f;
      float sR = 0.f;
      float fLH0 = 0.f;
      float fRH0 = 0.f;
      float fHL = 0.f;
      float fHR = 0.f;
      Angle turn = 0_deg;

      JointRequest jointRequest;
      Pose3f leftFoot;
      Pose3f rightFoot;

      float scaleForward = 1.f;
      while(true)
      {
        if(generateJointRequest(jointRequest, Pose2f(0.f, scaleForward * edgePoint.x(), edgePoint.y() * (1.f - i)), true, true, 1.f,
                                fL0, fR0, fL, fR, sL0, sR0, sL, sR, fLH0, fRH0, fHL, fHR, turn,
                                leftFoot, rightFoot))
        {
          Pose3f leftFootCopy = leftFoot;
          Pose3f rightFootCopy = rightFoot;
          if(useSafeSpace)
          {
            leftFoot.translation.x() += engine.stepSizeParameters.translationPolygonSafeRange.min; // some room to the balancer
            rightFoot.translation.x() += engine.stepSizeParameters.translationPolygonSafeRange.max;
            leftFootCopy.translation.x() += engine.stepSizeParameters.translationPolygonSafeRange.max;
            rightFootCopy.translation.x() += engine.stepSizeParameters.translationPolygonSafeRange.min;
          }
          if(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions) && InverseKinematic::calcLegJoints(leftFootCopy, rightFootCopy, Vector2f::Zero(), jointRequest, engine.theRobotDimensions))
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

  searchPolygonBorder(frontLeft, forwardAtMaxSide, forwardAt100, true, useSafeSpace);
  searchPolygonBorder(backRight, backwardAtMaxSide, backwardAt100, false, useSafeSpace);

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

bool WalkPhaseBase::generateJointRequest(JointRequest& jointRequest, const Pose2f& stepTarget,
                                         const bool convertStepTarget, const bool useIsLeftPhase,
                                         const float ratio, float fL0, float fR0, float fL, float fR,
                                         float sL0, float sR0, float sL, float sR,
                                         float& fLH0, float& fRH0, float fHL, float fHR,
                                         Angle& turn, Pose3f& leftFoot, Pose3f& rightFoot)
{
  Vector2f forwardAndSide;
  if(convertStepTarget)
  {
    const Vector2f hipOffset(0.f, isLeftPhase ? engine.theRobotDimensions.yHipOffset : -engine.theRobotDimensions.yHipOffset);
    forwardAndSide = (stepTarget.translation + hipOffset).rotated(-stepTarget.rotation * 0.5f) - 2.f * hipOffset + hipOffset.rotated(stepTarget.rotation * 0.5f);
  }
  else
    forwardAndSide = stepTarget.translation;

  if(useIsLeftPhase)
    calcFootOffsets(1.f, ratio, ratio, ratio, 1.f, 1.f, 1.f, fL0, fR0, fL, fR, sL0, sR0, sL, sR, fLH0, fRH0, fHL, fHR, turn, forwardAndSide.x(), forwardAndSide.y(), stepTarget.rotation);
  else
    calcFootOffsets(-1.f, ratio, ratio, ratio, 1.f, 1.f, 1.f, fR0, fL0, fR, fL, sR0, sL0, sR, sL, fRH0, fLH0, fHR, fHL, turn, forwardAndSide.x(), forwardAndSide.y(), stepTarget.rotation);

  engine.calcFeetPoses(fL, fR, sL, sR, fHL, fHR, turn, 0, 0, 0, 0, leftFoot, rightFoot, torsoShift);

  return InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions);
}

void WalkPhaseBase::initMaxRotationSpeedPerMotionStep(const Pose2f& targetStep)
{
  // dummy parameters
  float fL0 = forwardL0;
  float fR0 = forwardR0;
  float fL = forwardL;
  float fR = forwardR;
  float sL0 = sideL0;
  float sR0 = sideR0;
  float sL = sideL;
  float sR = sideR;
  float fLH0 = footHL0;
  float fRH0 = footHR0;
  float fHL = footHL;
  float fHR = footHR;

  Angle turn = 0;
  Angle lastTurn = 0;

  JointRequest jR, prevJR;
  Pose3f leftFoot;
  Pose3f rightFoot;

  auto oldSpeedRegulatorParams = speedRegulatorParams;

  for(float time = 0.f; time <= 0.25f; time = time + Constants::motionCycleTime > 0.25f && time < 0.25f ? 0.25f : time + Constants::motionCycleTime)
  {
    generateJointRequest(jR, targetStep, false, isLeftPhase, time / (engine.kinematicParameters.baseWalkPeriod / 1000.f),
                         fL0, fR0, fL, fR, sL0, sR0, sL, sR, fLH0, fRH0, fHL, fHR, turn,
                         leftFoot, rightFoot);

    if(time != 0.f)
    {
      const Joints::Joint supportHip = isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch;
      const Joints::Joint supportKnee = isLeftPhase ? Joints::rKneePitch : Joints::lKneePitch;
      const Joints::Joint supportAnkle = isLeftPhase ? Joints::rAnklePitch : Joints::lAnklePitch;
      auto& speedParams = speedRegulatorParams;
      speedParams.rotationSpeedHYP.max =
        std::max(speedParams.rotationSpeedHYP.max, Angle(std::abs(turn - lastTurn)));
      speedParams.rotationSpeedHip.max =
        std::max(speedParams.rotationSpeedHip.max,
                 Angle(std::abs(jR.angles[supportHip] - prevJR.angles[supportHip])));
      speedParams.rotationSpeedKnee.max =
        std::max(speedParams.rotationSpeedKnee.max,
                 Angle(std::abs(jR.angles[supportKnee] - prevJR.angles[supportKnee])));
      speedParams.rotationSpeedAnklePitch.max =
        std::max(speedParams.rotationSpeedAnklePitch.max,
                 Angle(std::abs(jR.angles[supportAnkle] - prevJR.angles[supportAnkle])));

      const std::vector<Angle> rollValues = { speedParams.rotationSpeedRoll.max,
                                              std::abs(jR.angles[Joints::rHipRoll] - prevJR.angles[Joints::rHipRoll]),
                                              std::abs(jR.angles[Joints::rAnkleRoll] - prevJR.angles[Joints::rAnkleRoll]),
                                              std::abs(jR.angles[Joints::lHipRoll] - prevJR.angles[Joints::lHipRoll]),
                                              std::abs(jR.angles[Joints::lAnkleRoll] - prevJR.angles[Joints::lAnkleRoll]),
                                            };
      speedParams.rotationSpeedRoll.max = *std::max_element(std::begin(rollValues), std::end(rollValues));
    }

    prevJR = jR;
    lastTurn = turn;
  }

  speedRegulatorParams.rotationSpeedHYP.max += oldSpeedRegulatorParams.rotationSpeedHYP.max;
  speedRegulatorParams.rotationSpeedRoll.max += oldSpeedRegulatorParams.rotationSpeedRoll.max;
  speedRegulatorParams.rotationSpeedHip.max += oldSpeedRegulatorParams.rotationSpeedHip.max;
  speedRegulatorParams.rotationSpeedKnee.max += oldSpeedRegulatorParams.rotationSpeedKnee.max;
  speedRegulatorParams.rotationSpeedAnklePitch.max += oldSpeedRegulatorParams.rotationSpeedAnklePitch.max;
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
  lastJointRequest = lastWalkPhaseDummy.lastJointRequest;

  afterWalkKickPhase = lastWalkPhaseDummy.walkKickStep.currentKick != WalkKicks::none && this->walkKickStep.currentKick == WalkKicks::none;
  noFastTranslationPolygonSteps = std::max(0, lastWalkPhaseDummy.noFastTranslationPolygonSteps - 1);
  armCompensationTilt = lastWalkPhaseDummy.armCompensationTilt;
  useSlowSupportFootHeightAfterKickInterpolation = lastWalkPhaseDummy.walkKickStep.useSlowSupportFootHeightAfterKickInterpolation;
  lastWalkPhaseKneeHipBalance = weightShiftStatus == weightDidShift ? lastWalkPhaseDummy.currentWalkPhaseKneeHipBalance : 0_deg;
  const Angle useSoleRotationOffsetSpeed = engine.theFrameInfo.getTimeSince(timeWhenLastKick) > engine.commonSpeedParameters.soleRotationOffsetSpeedAfterKickTime ? this->engine.commonSpeedParameters.soleRotationOffsetSpeed.max : this->engine.commonSpeedParameters.soleRotationOffsetSpeed.min;
  lastWalkPhaseKneeHipBalance -= Rangea(-useSoleRotationOffsetSpeed * Constants::motionCycleTime, useSoleRotationOffsetSpeed * Constants::motionCycleTime).limit(lastWalkPhaseKneeHipBalance);
  gyroStateTimestamp = lastWalkPhaseDummy.gyroStateTimestamp;

  walkStepAdjustment.init(lastWalkPhaseDummy.walkStepAdjustment, resetWalkStepAdjustment, this->engine.theFrameInfo, this->engine.walkStepAdjustmentParams.unstableWalkThreshold);
  walkStepAdjustment.lastWasKneeBalance = lastWalkPhaseDummy.kneeBalance > 0_deg;
}

void WalkPhaseBase::constructorWalkCase(const MotionPhase& lastPhase, const Pose2f& useStepTarget, Pose2f& lastStepTarget, const bool standRequested)
{
  const auto& lastWalkPhaseDummy = static_cast<const WalkPhaseBase&>(lastPhase);
  ASSERT(lastWalkPhaseDummy.walkState != standing);

  weightShiftStatus = (lastWalkPhaseDummy.supportSwitchInfo.isFootSupportSwitch && lastWalkPhaseDummy.isLeftPhase != (engine.theFootSupport.support < 0.f)) ||
                      (!lastWalkPhaseDummy.supportSwitchInfo.isFootSupportSwitch && lastWalkPhaseDummy.isLeftPhase != (engine.theFootSupport.support > 0.f)) ? weightDidShift : weightDidNotShift;

  isLeftPhase = engine.theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget);

  lastStepTarget = lastWalkPhaseDummy.step;
  doBalanceSteps = std::max(0, lastWalkPhaseDummy.doBalanceSteps - 1);
  walkState = standRequested ? stopping : walking;

  // init members, to ensure isStandingPossible() has all information it needs
  constructorHelperInitVariablesLastPhase(lastWalkPhaseDummy, false);

  // When slowing down, the robots have some momentum left which causes it to tilt forward. The first step itself is fine, the one afterwards is problematic.
  if(lastStepTarget.translation.x() - 20.f > useStepTarget.translation.x())
  {
    walkStepAdjustment.kneeHipBalanceCounter = std::max(walkStepAdjustment.kneeHipBalanceCounter, lastStepTarget.translation.x() - 40.f > useStepTarget.translation.x() ? 2.f : 1.f);
    walkStepAdjustment.isForwardBalance = true;
    walkStepAdjustment.forwardBalanceWasActive = false;
  }

  // if robot is stopping, we need 1-2 walk phases for it
  if(walkState == stopping && lastWalkPhaseDummy.walkState == stopping)
  {
    if(isStandingPossible(lastWalkPhaseDummy.forwardL, lastWalkPhaseDummy.forwardR, lastWalkPhaseDummy.lastPrevSideL,
                          lastWalkPhaseDummy.lastPrevSideR, lastWalkPhaseDummy.turnRL, lastWalkPhaseDummy.walkStepAdjustment.lastLeftAdjustmentX,
                          lastWalkPhaseDummy.walkStepAdjustment.lastRightAdjustmentX, lastWalkPhaseDummy.walkStepAdjustment.highestAdjustmentX,
                          lastWalkPhaseDummy.sideHipShift, false))
    {
      walkState = standing;
      stepDuration = stepDuration = engine.stepDurationSpeedTarget(engine.getSideSpeed(0.f));
      stepHeightDuration = engine.stepDurationSpeedTarget(engine.getSideSpeed(0.f));
    }
  }

  if(walkKickStep.numOfBalanceSteps > 0)
    doBalanceSteps = walkKickStep.numOfBalanceSteps;

  WalkKickStep::OverrideFoot left(WalkKickStep::OverrideFoot::none);
  WalkKickStep::OverrideFoot right(WalkKickStep::OverrideFoot::none);
  if(!lastWalkPhaseDummy.isLeftPhase && isLeftPhase)
  {
    right = lastWalkPhaseDummy.walkKickStep.overrideOldSwingFoot;
    left = lastWalkPhaseDummy.walkKickStep.overrideOldSupportFoot;
  }
  else if(lastWalkPhaseDummy.isLeftPhase && !isLeftPhase)
  {
    left = lastWalkPhaseDummy.walkKickStep.overrideOldSwingFoot;
    right = lastWalkPhaseDummy.walkKickStep.overrideOldSupportFoot;
  }

  constructorInitWithJointRequest(useStepTarget, left, right);

  if(walkState != standing)
  {
    lastSideStep = lastWalkPhaseDummy.sideStep;
    prevSideL = sideL0;
    prevSideR = sideR0;
    prevTurn = lastWalkPhaseDummy.prevTurn;
  }
  if(engine.theGroundContactState.contact)    // should not be !standRequest, because robot could be stuck at a goal post
  {
    if(weightShiftStatus == weightDidNotShift && lastWalkPhaseDummy.weightShiftStatus == emergencyStand && lastWalkPhaseDummy.supportSwitchInfo.isFeetStepAbort)
      weightShiftStatus = emergencyStep;
    else if(weightShiftStatus == weightDidNotShift)
      weightShiftStatus = emergencyStand;
    if(weightShiftStatus == weightDidShift)
    {
      engine.filteredGyroX = (1.f - engine.balanceParameters.gyroLowPassRatio) * engine.theInertialData.gyro.x();
      engine.filteredGyroY = (1.f - engine.balanceParameters.gyroLowPassRatio) * engine.theInertialData.gyro.y();
    }
  }
  engine.theWalkStepData.updateCounter(!lastWalkPhaseDummy.supportSwitchInfo.isFootSupportSwitch);

  // calibrate step height duration time
  engine.theWalkLearner.updateStepDuration(lastWalkPhaseDummy.step, lastWalkPhaseDummy.tBase, engine.kinematicParameters.baseWalkPeriod / 1000.f);
}

void WalkPhaseBase::constructorStandCase(const bool standRequested, const MotionPhase& lastPhase, const Pose2f& useStepTarget)
{
  const auto& lastWalkPhaseDummy = static_cast<const WalkPhaseBase&>(lastPhase);
  constructorHelperInitVariablesLastPhase(lastWalkPhaseDummy, true);
  walkState = standRequested ? standing : starting;
  weightShiftStatus = weightDidShift;

  isLeftPhase = engine.theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget);
  constructorInitWithJointRequest(Pose2f(0.f, 0.f, 0.f), WalkKickStep::OverrideFoot::request, WalkKickStep::OverrideFoot::request);
}

void WalkPhaseBase::constructorAfterKickOrGetUpCase(const MotionPhase& lastPhase)
{
  ASSERT(!engine.translationPolygonAfterKick.empty());
  afterKickWalkPhase = true;
  walkState = starting;
  weightShiftStatus = weightDidShift;
  isLeftPhase = engine.theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());

  constructorInitWithJointRequest(Pose2f(0.f, 0.f, 0.f), WalkKickStep::OverrideFoot::request, WalkKickStep::OverrideFoot::request);

  armCompensationAfterKick = 1.f;
  const RobotModel lastRobotModel(engine.theJointRequest, engine.theRobotDimensions, engine.theMassCalibration);

  forwardStep = 0.f;
  sideStep = -(isLeftPhase ? lastRobotModel.soleRight.translation.y() + 50.f : lastRobotModel.soleLeft.translation.y() - 50.f);
  sideStep = Rangef(isLeftPhase ? 0.f : -1000.f, isLeftPhase ? 1000.f : 0.f).limit(sideStep);

  const Vector2f target(forwardStep, sideStep);
  Vector2f p1;
  if(!Geometry::isPointInsideConvexPolygon(engine.translationPolygonAfterKick.data(), static_cast<int>(engine.translationPolygonAfterKick.size()), target))
  {
    VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(engine.translationPolygonAfterKick, Geometry::Line(Vector2f(0.f, 0.f),
                                                           target.normalized()), p1, false));
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
  engine.calcFeetPoses(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFoot, rightFoot, torsoShift); // current request
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
    if(joint < Joints::firstArmJoint || joint == Joints::lWristYaw || joint == Joints::lHand || joint == Joints::rWristYaw || joint == Joints::rHand)
      continue;
    if(startJointAngles.angles[joint] == JointAngles::off || startJointAngles.angles[joint] == JointAngles::ignore)
      startJointAngles.angles[joint] = engine.theJointAngles.angles[joint];
    const Angle dif = startJointAngles.angles[joint] - request.angles[joint];
    const float factor = joint >= Joints::firstArmJoint && joint < Joints::firstLeftLegJoint ? 2.f : 1.f; // arms are factored more than all other joints
    jointDiff = std::max(jointDiff, Angle(std::abs(dif * factor)));
  }
  standInterpolationDuration = std::max(engine.standHighInterpolationDuration, jointDiff / engine.standInterpolationVelocity * 1000.f);
  armCompensationAfterKick = 1.f;
  leftArmInterpolationStart = engine.theFrameInfo.time;
  rightArmInterpolationStart = engine.theFrameInfo.time;
}

void WalkPhaseBase::constructorHelperInitWalkPhase(const Pose2f& useStepTarget, const Pose2f& lastStepTarget, const bool afterKickOrGetUp, const bool standRequested, const MotionPhase& lastPhase)
{
  // Duration of the walking step
  stepDuration = stepDurationSide = !useSlowSupportFootHeightAfterKickInterpolation ? engine.stepDurationSpeedTarget(0.f) : (engine.kinematicParameters.baseWalkPeriod * (1.f + engine.kinematicParameters.sidewaysWalkHeightPeriodIncreaseFactor)) / 1000.f;

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
  useSpeedForHeightDuration = std::max(engine.theWalkLearner.stepHeightDurationOffset, std::max(walkKickStep.useSlowSwingFootHeightInterpolation, useSpeedForHeightDuration));

  // the swing height duration
  stepHeightDuration = engine.stepDurationSpeedTarget(useSpeedForHeightDuration);

  // normal walk case
  if(!afterKickOrGetUp && walkKickStep.currentKick == WalkKicks::none)
  {
    const Pose2f usedStepTarget = standRequested ? Pose2f() : useStepTarget;
    const Pose2f nextStep = convertStep(usedStepTarget, 0);

    forwardStep = nextStep.translation.x();
    sideStep = nextStep.translation.y();
    turnStep = usedStepTarget.rotation;

    step = usedStepTarget;
  }
  // in walk kick case
  else if(!afterKickOrGetUp)
  {
    noFastTranslationPolygonSteps = engine.stepSizeParameters.noFastTranslationPolygonStepsNumber;
    step = useStepTarget;
    for(size_t i = 0; i < walkKickStep.keyframe.size(); i++)
    {
      // Main Step
      const Pose2f& useStepTarget = this->walkKickStep.keyframe[i].stepTarget;
      Pose2f nextStep = convertStep(useStepTarget, 0);
      this->walkKickStep.keyframe[i].stepTargetConverted.translation = nextStep.translation;
      this->walkKickStep.keyframe[i].stepTargetConverted.rotation = this->walkKickStep.keyframe[i].stepTarget.rotation;

      // Offset swing step
      const Pose2f& useStepTargetSwing = this->walkKickStep.keyframe[i].stepTargetSwing;
      nextStep = convertStep(useStepTargetSwing, 0);
      this->walkKickStep.keyframe[i].stepTargetSwingConverted.translation = nextStep.translation;
      this->walkKickStep.keyframe[i].stepTargetSwingConverted.rotation = this->walkKickStep.keyframe[i].stepTarget.rotation;
    }
    forwardStep = this->walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.translation.x();
    sideStep = this->walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.translation.y();
    turnStep = this->walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.rotation;
  }
  maxFootHeight = engine.kinematicParameters.baseFootLift * (walkState == starting ? engine.commonSpeedParameters.reduceSwingHeightStartingFactor * (1.f - std::abs(sideSpeed) / engine.configuredParameters.maxSpeed.translation.y()) + std::abs(sideSpeed) / engine.configuredParameters.maxSpeed.translation.y() : 1.f) * walkKickStep.increaseSwingHeightFactor;

  // After a big side step and finishing the closing of the feet, there is still a lot of momentum left.
  // Shift the hip in the y-axis to compensate this momentum, but only if the robot would continue walking straight.
  if(lastPhase.type == MotionPhase::walk)
  {
    const SideStabilizeParameters& ssp = engine.sideStabilizeParameters;

    // Side correction to slow down momentum
    float sideLEnd;
    float sideREnd;
    float& sideSupport = isLeftPhase ? sideREnd : sideLEnd;
    float& sideSwing = !isLeftPhase ? sideREnd : sideLEnd;
    sideSupport = -sideStep * engine.kinematicParameters.sidewaysHipShiftFactor;
    sideSwing = sideStep * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor);

    const float requestedStartDiff = sideL0 - sideR0;
    const float measuredStartDiff = measuredSideL0 - measuredSideR0;
    const float endDiff = sideLEnd - sideREnd;
    const float ratio = mapToRange(std::max(requestedStartDiff - endDiff, measuredStartDiff - endDiff), ssp.sideHipShiftStepSizeRange.min, ssp.sideHipShiftStepSizeRange.max, 0.f, 1.f);
    const float useSideStep = sideStep;
    const float stepRatio = 1.f - mapToRange(useSideStep * (isLeftPhase ? 1.f : -1.f), 0.f, ssp.maxSideHipShiftStepSize, 0.f, 1.f);
    sideHipShift = ratio * stepRatio * ssp.maxSideHipShift * (!isLeftPhase ? 1.f : -1.f);

    const float maxSideSpeed = engine.configuredParameters.walkSpeedParams.maxSpeed.translation.y() * Constants::motionCycleTime;
    sideAcc = sideSupport - (isLeftPhase ? sideR0 : sideL0) > 0.f ? -maxSideSpeed : maxSideSpeed;
  }
};

void WalkPhaseBase::constructorInitWithJointRequest(const Pose2f& useStepTarget, const WalkKickStep::OverrideFoot left, const WalkKickStep::OverrideFoot right)
{
  JointAngles request = engine.theJointRequest;

  if(left == WalkKickStep::OverrideFoot::measured || right == WalkKickStep::OverrideFoot::measured)
  {
    for(std::size_t joint = left == WalkKickStep::OverrideFoot::measured ? Joints::firstLeftLegJoint : Joints::firstRightLegJoint;
        joint < (right == WalkKickStep::OverrideFoot::measured ? Joints::numOfJoints : Joints::firstRightLegJoint); joint++)
      request.angles[joint] = engine.theJointAngles.angles[joint];

    request.angles[Joints::lHipYawPitch] = engine.theJointAngles.angles[Joints::lHipYawPitch];
    request.angles[Joints::rHipYawPitch] = engine.theJointAngles.angles[Joints::rHipYawPitch];
  }

  // Adjust roll joints
  {
    const float hipOffset = engine.theRobotDimensions.yHipOffset;
    const float sideStartLeft = engine.theRobotModel.soleLeft.translation.y() - hipOffset;
    const float sideStartRight = engine.theRobotModel.soleRight.translation.y() + hipOffset;

    float sideChangeLeft;
    float sideChangeRight;

    if(isLeftPhase)
    {
      sideChangeLeft = (useStepTarget.translation.y() * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor) - sideStartLeft);
      sideChangeRight = (-useStepTarget.translation.y() * engine.kinematicParameters.sidewaysHipShiftFactor - sideStartRight);
    }
    else
    {
      sideChangeLeft = (-useStepTarget.translation.y() * engine.kinematicParameters.sidewaysHipShiftFactor - sideStartLeft);
      sideChangeRight = (useStepTarget.translation.y() * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor) - sideStartRight);
    }

    const Angle leftBestHipRoll = sideChangeLeft > 0 ? std::max(request.angles[Joints::lHipRoll], engine.theJointAnglePred.angles[Joints::lHipRoll]) : std::min(request.angles[Joints::lHipRoll], engine.theJointAnglePred.angles[Joints::lHipRoll]);
    const Angle rightBestHipRoll = sideChangeRight > 0 ? std::max(request.angles[Joints::rHipRoll], engine.theJointAnglePred.angles[Joints::rHipRoll]) : std::min(request.angles[Joints::rHipRoll], engine.theJointAnglePred.angles[Joints::rHipRoll]);
    const Angle maxAngleChange = 3_deg;
    const Angle leftBestClipHipRoll = Rangea(request.angles[Joints::lHipRoll] - maxAngleChange, request.angles[Joints::lHipRoll] + maxAngleChange).limit(leftBestHipRoll);
    const Angle rightBestClipHipRoll = Rangea(request.angles[Joints::rHipRoll] - maxAngleChange, request.angles[Joints::rHipRoll] + maxAngleChange).limit(rightBestHipRoll);

    const float leftRatio = Rangef::ZeroOneRange().limit(std::abs(sideChangeLeft) / 20.f);
    const float rightRatio = Rangef::ZeroOneRange().limit(std::abs(sideChangeRight) / 20.f);

    request.angles[Joints::lHipRoll] = request.angles[Joints::lHipRoll] * (1.f - leftRatio) + leftRatio * leftBestClipHipRoll;
    request.angles[Joints::rHipRoll] = request.angles[Joints::rHipRoll] * (1.f - rightRatio) + rightRatio * rightBestClipHipRoll;
  }

  // Adjust start hipPitch and anklePitch for swing foot
  if(walkStepAdjustment.delta > 0.f || (walkStepAdjustment.delta == 0.f && useStepTarget.translation.x() > 0.f))
  {
    if(((isLeftPhase && left == WalkKickStep::OverrideFoot::none) ||
        (!isLeftPhase && right == WalkKickStep::OverrideFoot::none)) && engine.theGroundContactState.contact)
    {
      float ratio;
      if(walkStepAdjustment.delta > 0.f)
        ratio = Rangef::ZeroOneRange().limit(walkStepAdjustment.delta / engine.highDeltaScale);
      else
        ratio = mapToRange(float(engine.theInertialData.angle.y()), 4_deg - armCompensationTilt[0], 10_deg - armCompensationTilt[0], 0.f, 1.f) * Rangef::ZeroOneRange().limit(useStepTarget.translation.x() / 30.f);
      lastWalkPhaseKneeHipBalance = 0.f;
      const Joints::Joint jointHip = isLeftPhase ? Joints::lHipPitch : Joints::rHipPitch;
      const Joints::Joint jointAnkle = isLeftPhase ? Joints::lAnklePitch : Joints::rAnklePitch;
      request.angles[jointHip] = std::min(request.angles[jointHip], Angle(request.angles[jointHip] * (1.f - ratio) + engine.theJointAngles.angles[jointHip] * ratio));
      request.angles[jointAnkle] = request.angles[jointAnkle] * (1.f - ratio) + engine.theJointAngles.angles[jointAnkle] * ratio;
    }

    constructorArmCompensation(request, armCompensationTilt[0]);
    if(isLeftPhase && left == WalkKickStep::OverrideFoot::none)
    {
      soleRotationYL = std::min(0_deg, soleRotationYL);
      soleRotationXL = 0_deg;
    }
    else if(!isLeftPhase && right == WalkKickStep::OverrideFoot::none)
    {
      soleRotationYR = std::min(0_deg, soleRotationYR);
      soleRotationXR = 0_deg;
    }
  }
  // Adjust start hip and ankle for support foot
  // Good idea on paper, bad idea on the playing field. To many other factors exist, which make this code worse
  else
  {
    if(((isLeftPhase && left == WalkKickStep::OverrideFoot::none) ||
        (!isLeftPhase && right == WalkKickStep::OverrideFoot::none)) && engine.theGroundContactState.contact)
    {
      float ratio;
      if(walkStepAdjustment.delta < 0.f)
        ratio = Rangef::ZeroOneRange().limit(walkStepAdjustment.delta / -engine.highDeltaScale);
      else
        ratio = (1.f - mapToRange(float(engine.theInertialData.angle.y()), -6_deg - armCompensationTilt[0] / 2.f, -1_deg - armCompensationTilt[0] / 2.f, 0.f, 1.f)) * Rangef::ZeroOneRange().limit(useStepTarget.translation.x() / -15.f);
      lastWalkPhaseKneeHipBalance = 0.f;
      const Joints::Joint jointHipSwing = isLeftPhase ? Joints::lHipPitch : Joints::rHipPitch;
      const Joints::Joint jointHipSupport2 = !isLeftPhase ? Joints::lHipPitch : Joints::rHipPitch;
      request.angles[jointHipSwing] = std::max(request.angles[jointHipSwing], Angle(request.angles[jointHipSwing] * (1.f - ratio) + engine.theJointAngles.angles[jointHipSwing] * ratio));
      request.angles[jointHipSupport2] = std::min(request.angles[jointHipSupport2], Angle(request.angles[jointHipSupport2] * (1.f - ratio) + (engine.theJointAngles.angles[jointHipSupport2] + engine.theJointPlay.jointState[jointHipSupport2].velocity) * ratio));
    }

    constructorArmCompensation(request, armCompensationTilt[0]);
    if(isLeftPhase && left == WalkKickStep::OverrideFoot::none)
    {
      soleRotationYL = std::max(0_deg, soleRotationYL);
      soleRotationXL = 0_deg;
    }
    else if(!isLeftPhase && right == WalkKickStep::OverrideFoot::none)
    {
      soleRotationYR = std::max(0_deg, soleRotationYR);
      soleRotationXR = 0_deg;
    }
  }

  if(engine.theFrameInfo.getTimeSince(timeWhenLastKick) == 0)
  {
    if(isLeftPhase)
    {
      soleRotationYL = -engine.theInertialData.angle.y() * 0.75f;
      soleRotationXL = -engine.theInertialData.angle.x() * 0.75f;
    }
    else
    {
      soleRotationYR = -engine.theInertialData.angle.y() * 0.75f;
      soleRotationXR = -engine.theInertialData.angle.x() * 0.75f;
    }
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

void WalkPhaseBase::calcJointsHelperFootSoleRotations(const JointRequest& jointRequest, const Angle hipRotation)
{
  // Reset feet rotation offsets back to 0
  const Angle useSoleRotationOffsetSpeed = engine.theFrameInfo.getTimeSince(timeWhenLastKick) > engine.commonSpeedParameters.soleRotationOffsetSpeedAfterKickTime ?
                                           this->engine.commonSpeedParameters.soleRotationOffsetSpeed.max :
                                           this->engine.commonSpeedParameters.soleRotationOffsetSpeed.min;
  const Rangea limitAnkleOffset(-useSoleRotationOffsetSpeed * Constants::motionCycleTime, useSoleRotationOffsetSpeed * Constants::motionCycleTime);
  const Angle soleYLOld = soleRotationYL;
  const Angle soleYROld = soleRotationYR;
  const Angle soleXLOld = soleRotationXL;
  const Angle soleXROld = soleRotationXR;
  soleRotationYL -= limitAnkleOffset.limit(soleRotationYL);
  soleRotationXL -= limitAnkleOffset.limit(soleRotationXL);
  soleRotationYR -= limitAnkleOffset.limit(soleRotationYR);
  soleRotationXR -= limitAnkleOffset.limit(soleRotationXR);

  const Rangea limitAnkleOffsetFast(-120_deg * Constants::motionCycleTime, 120_deg * Constants::motionCycleTime);

  const Angle oldKneeBalance = kneeBalance;
  kneeBalance -= limitAnkleOffsetFast.limit(kneeBalance);

  // Add sole rotation compensation
  if(walkState != standing && engine.theGroundContactState.contact)
  {
    const float sideStart = isLeftPhase ? sideL0 : sideR0;
    const float sideChangeSigned = (sideStep * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor) - sideStart) * (isLeftPhase ? 1.f : -1.f);

    // get predicted torsorot
    const RobotModel balanced(jointRequest, engine.theRobotDimensions, engine.theMassCalibration);
    walkStepAdjustment.modifySwingFootRotation(isLeftPhase ? soleRotationYL : soleRotationYR,
                                               isLeftPhase ? soleYLOld : soleYROld,
                                               isLeftPhase ? soleRotationXL : soleRotationXR,
                                               isLeftPhase ? soleXLOld : soleXROld,
                                               engine.theInertialData.angle.x(), engine.theInertialData.angle.y(),
                                               Rangef::ZeroOneRange().limit(tBase / stepDuration),
                                               hipRotation, isLeftPhase,
                                               engine.soleRotationParameter,
                                               !isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft,
                                               sideChangeSigned, kneeBalance, oldKneeBalance,  engine.theInertialData.gyro.y());

    if(kneeBalance > 0.f)
      sideStep = isLeftPhase ? std::max(sideStep, engine.soleRotationParameter.minSideStepSize) : std::max(sideStep, -engine.soleRotationParameter.minSideStepSize);
  }
}

void WalkPhaseBase::calcJointsHelperInterpolateStanding(const MotionRequest& motionRequest)
{
  if(standFactor != 1.f)
    timeWhenStandHighBegan = engine.theFrameInfo.time;

  wasStandingStillOnce |= engine.theFrameInfo.getTimeSince(engine.theIMUValueState.gyroValues.stableSinceTimestamp) >= engine.standHighNotMovingTime;
  const bool highStandRequested = !motionRequest.shouldInterceptBall && motionRequest.motion == MotionRequest::stand && motionRequest.standHigh && wasStandingStillOnce;
  if(standFactor < 0.f)
  {
    const float nextFactor = std::min((std::asin((standFactor + 1.f) * 2.f - 1.f) + Constants::pi_2) / Constants::pi + 1000.f * Constants::motionCycleTime / standInterpolationDuration, 1.f);
    standFactor = (std::sin(-Constants::pi_2 + Constants::pi * nextFactor) + 1.f) * 0.5f - 1.f;
    standFactor = std::min(standFactor, 0.f);
  }
  else if(highStandRequested)
  {
    const float nextFactor = std::min((std::asin(standFactor * 2.f - 1.f) + Constants::pi_2) / Constants::pi + 1000.f * Constants::motionCycleTime / engine.standHighInterpolationDuration, 1.f);
    standFactor = (std::sin(-Constants::pi_2 + Constants::pi * nextFactor) + 1.f) * 0.5f;
    standFactor = std::min(standFactor, 1.f);
  }
  else
  {
    const float nextFactor = std::max(0.f, std::min((std::asin(standFactor * 2.f - 1.f) + Constants::pi_2) / Constants::pi - 1000.f * Constants::motionCycleTime / engine.standHighInterpolationDuration, 1.f));
    standFactor = (std::sin(-Constants::pi_2 + Constants::pi * nextFactor) + 1.f) * 0.5f;
    standFactor = std::max(standFactor, 0.f);
  }

  const float standFactor01 = std::max(standFactor, 0.f);
  forwardL0 = forwardR0 = standFactor01 * torsoShift;
  footHL0 = footHR0 = -standFactor01 *
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
  ASSERT(!engine.translationPolygonBigNotClipped.empty());
  const RotationMatrix& rot = engine.theRobotStableState.predictedTorsoRotation;
  const Vector3f sole = rot.inverse() * (isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft).translation;
  const Vector3f comInTorso = rot * (Vector3f() << (rot.inverse() * engine.theRobotStableState.lightCenterOfMass).head<2>(), sole.z()).finished();
  const Vector2f diff = comInTorso.head<2>();

  const std::vector<Vector2f>& polygon = engine.translationPolygonBigNotClipped;

  const float maxSide = std::max(std::abs(sideL), std::abs(sideR));
  const float sideRatioBackward = mapToRange(maxSide * 2.f, polygon[polygon.size() - 2].y(), polygon.back().y(), 0.f, 1.f);
  const float sideRatioForward = mapToRange(maxSide * 2.f, polygon[1].y(), polygon[0].y(), 0.f, 1.f);
  // The polygon already knows based on the base torso offset what the limits are
  // With the dynamic shift, this must be slightly corrected
  Rangef clipForward(mapToRange(sideRatioBackward, 0.f, 1.f, polygon[polygon.size() - 2].x() / 2.f, polygon[polygon.size() - 1].x() / 2.f) + torsoShift - engine.kinematicParameters.torsoOffset,
                     mapToRange(sideRatioForward, 0.f, 1.f, polygon[1].x() / 2.f, polygon[0].x() / 2.f) + torsoShift - engine.kinematicParameters.torsoOffset);

  if(walkKickStep.currentKick == WalkKicks::none && (!clipForward.isInside(walkStepAdjustment.lastLeft.translation.x() + walkStepAdjustment.lastLeftAdjustmentX) || !clipForward.isInside(walkStepAdjustment.lastRight.translation.x() + walkStepAdjustment.lastRightAdjustmentX)))
  {
    freezeSideMovement = prevSideL <= sideL && prevSideR >= sideR; // feet are moving outwards
    sideL = std::min(prevSideL, sideL);
    sideR = std::max(prevSideR, sideR); // we assume sideR is big negative, and the max is closer to 0

    engine.calcFeetPoses(forwardL, forwardR, sideL, sideR, footHL, footHR, turnRL, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFoot, rightFoot, torsoShift); // current request

    const float maxSide = std::max(std::abs(sideL), std::abs(sideR));
    const float sideRatio = mapToRange(maxSide * 2.f, polygon[1].y(), polygon[0].y(), 0.f, 1.f);
    clipForward = Rangef(mapToRange(sideRatio, 0.f, 1.f, polygon[polygon.size() - 2].x() / 2.f, polygon[polygon.size() - 1].x() / 2.f),
                         mapToRange(sideRatio, 0.f, 1.f, polygon[1].x() / 2.f, polygon[0].x() / 2.f));
  }
  else
    freezeSideMovement = false;

  Rangef clipForwardSupport = clipForward;
  clipForward.min -= torsoShift;
  clipForward.max -= torsoShift;

  // Make sure we do not clip too much in case the feet are far off from the last walk step (e.g. after an InWalkKick)
  clipForward = Rangef(std::min(clipForward.min, std::min(leftFoot.translation.x(), rightFoot.translation.x())),
                       std::max(clipForward.max, std::max(leftFoot.translation.x(), rightFoot.translation.x())));

  walkStepAdjustment.addBalance(leftFoot, rightFoot, tBase / stepDuration, diff, engine.theFootOffset, clipForward,
                                isLeftPhase, engine.theFootSupport, engine.theFsrData, engine.theFrameInfo,
                                armCompensationTilt[0], walkState != standing, // should step adjustment be active?
                                engine.commonSpeedParameters.reduceWalkingSpeedStepAdjustmentSteps, ball, engine.clipAtBallDistanceX,
                                engine.theGroundContactState.contact, engine.walkStepAdjustmentParams);

  if(engine.theFrameInfo.getTimeSince(annotationTimestamp) > 10000 && (std::abs(walkStepAdjustment.lastLeftAdjustmentX) > engine.walkStepAdjustmentParams.unstableWalkThreshold || std::abs(walkStepAdjustment.lastRightAdjustmentX) > engine.walkStepAdjustmentParams.unstableWalkThreshold))
  {
    annotationTimestamp = engine.theFrameInfo.time;
    ANNOTATION("WalkingEngine", "Step adjusted");
  }
}

JointAngles WalkPhaseBase::addGyroBalance(JointRequest& jointRequest, const JointAngles& lastRequest)
{
  Rangea changesInRequest[Joints::numOfJoints - Joints::firstLegJoint];
  for(std::size_t i = Joints::firstLegJoint; i < Joints::numOfJoints; i++)
  {
    const Angle change = lastRequest.angles[i] - jointRequest.angles[i];
    changesInRequest[i - Joints::firstLegJoint] = Rangea(std::min(Angle(change - 0.5_deg), -0.5_deg), std::max(Angle(change + 0.5_deg), 0.5_deg));
  }

  std::array<Angle, Joints::numOfJoints> balancingValues;
  balancingValues.fill(0);

  // Sagittal balance
  const Angle balanceAdjustmentY = (walkState == standing ? 0.5f : 1.f) * engine.filteredGyroY *
                                   (engine.filteredGyroY > 0
                                    ? engine.balanceParameters.gyroForwardBalanceFactor
                                    : engine.balanceParameters.gyroBackwardBalanceFactor); // adjust ankle tilt in proportion to filtered gyroY
  balancingValues[isLeftPhase ? Joints::rAnklePitch : Joints::lAnklePitch] += balanceAdjustmentY;
  PLOT("module:WalkingEngine:Data:sagittal", balanceAdjustmentY.toDegrees());

  // Balance with the hipPitch for one motion phase after a kick phase.
  // TODO eval if this can be removed
  if(walkKickStep.currentKick != WalkKicks::none || afterWalkKickPhase || engine.theFrameInfo.getTimeSince(timeWhenLastKick) < engine.commonSpeedParameters.soleRotationOffsetSpeedAfterKickTime)
  {
    const Angle balancingValue = engine.filteredGyroY *
                                 (engine.filteredGyroY > 0
                                  ? engine.balanceParameters.gyroForwardBalanceFactorHipPitch.y() * walkStepAdjustment.hipBalanceIsSafeBackward
                                  : engine.balanceParameters.gyroForwardBalanceFactorHipPitch.x()) * walkStepAdjustment.hipBalanceIsSafeForward;
    PLOT("module:WalkingEngine:Data:hipBalancing", balancingValue.toDegrees());
    balancingValues[isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += balancingValue;
  }

  // Lateral balance
  if(walkState == standing)
  {
    const Angle balanceAdjustmentX = engine.filteredGyroX * engine.balanceParameters.gyroSidewaysBalanceFactor;
    balancingValues[Joints::lAnkleRoll] += balanceAdjustmentX;
    balancingValues[Joints::rAnkleRoll] += balanceAdjustmentX;
    balancingValues[!isLeftPhase ? Joints::rAnklePitch : Joints::lAnklePitch] += balanceAdjustmentY; // add other anklePitch
    PLOT("module:WalkingEngine:Data:lateral", balanceAdjustmentX.toDegrees());
  }

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
    const Rangef balanceRange(walkStepAdjustment.isBackwardBalance ? -engine.configuredParameters.maxGyroBalanceKneeValue : 0_deg, walkStepAdjustment.isForwardBalance || tBase > stepDuration ? engine.configuredParameters.maxGyroBalanceKneeValue : 0_deg);

    const float useIsSafeFactor = (engine.filteredGyroY > 0.f ? walkStepAdjustment.hipBalanceIsSafeForward : walkStepAdjustment.hipBalanceIsSafeBackward * (tBase > stepDuration ? 1.f : walkStepAdjustment.balanceComIsForward));
    currentWalkPhaseKneeHipBalance = balanceRange.limit(engine.filteredGyroY * engine.balanceParameters.gyroBalanceKneeBalanceFactor) * useIsSafeFactor;
    balancingValues[isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += currentWalkPhaseKneeHipBalance;
    balancingValues[isLeftPhase ? Joints::rKneePitch : Joints::lKneePitch] += currentWalkPhaseKneeHipBalance;
  }
  PLOT("module:WalkingEngine:Data:kneeHipBalance", currentWalkPhaseKneeHipBalance.toDegrees());
  balancingValues[!isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += lastWalkPhaseKneeHipBalance;
  balancingValues[!isLeftPhase ? Joints::rKneePitch : Joints::lKneePitch] += lastWalkPhaseKneeHipBalance;

  // Apply all balancing values, but interpolate to 0 if there is no ground contact
  float noGroundContactFactor = 1.f;
  if(!engine.theGroundContactState.contact)
    noGroundContactFactor *= Rangef::ZeroOneRange().limit(1.f - engine.theFrameInfo.getTimeSince(engine.theGroundContactState.lastGroundContactTimestamp) / 100.f);

  JointAngles appliedChanges;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if((joint == Joints::lHipPitch || joint == Joints::rHipPitch || joint == Joints::lKneePitch || joint == Joints::rKneePitch))
    {
      appliedChanges.angles[joint] += changesInRequest[joint - Joints::firstLegJoint].limit(balancingValues[joint]) * noGroundContactFactor;
      jointRequest.angles[joint] += appliedChanges.angles[joint];
    }
    else
    {
      appliedChanges.angles[joint] += balancingValues[joint] * noGroundContactFactor;
      jointRequest.angles[joint] += appliedChanges.angles[joint];
    }
  }

  // In highStand, the robot shall tilt a bit forward
  jointRequest.angles[Joints::lHipPitch] += std::max(standFactor, 0.f) * engine.standHighTorsoPitch;
  jointRequest.angles[Joints::rHipPitch] += std::max(standFactor, 0.f) * engine.standHighTorsoPitch;

  return appliedChanges;
}

bool WalkPhaseBase::isStandingPossible(const float forwardL, const float forwardR, const float sideL, const float sideR, const float turnRL,
                                       const float balanceAdjustmentLeft, const float balanceAdjustmentRight, const float highestAdjustmentX,
                                       const float sideHipShift, const bool isDive) const
{
  // when walking normally with ground contact, one step to go into neutral is enough. when picked up, execute neutral walk phases until the legs are close together
  bool areFeetCloseToOrigin = true;
  const Pose2f& thresholds = isDive ? engine.configuredParameters.thresholdDiveStandTransition : engine.configuredParameters.thresholdStopStandTransition;
  if(walkState != stopping || !engine.theGroundContactState.contact || sideHipShift != 0.f)
  {
    areFeetCloseToOrigin = std::abs(forwardL + balanceAdjustmentLeft) < thresholds.translation.x() &&
                           std::abs(forwardR + balanceAdjustmentRight) < thresholds.translation.x() &&
                           std::abs(sideL) < thresholds.translation.y() &&
                           std::abs(sideR) < thresholds.translation.y() &&
                           std::abs(turnRL) < thresholds.rotation;
  }
  else if(engine.theGroundContactState.contact)
    areFeetCloseToOrigin = !engine.blockStoppingWithStepAdjustment || highestAdjustmentX == 0.f;

  return areFeetCloseToOrigin;
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

void WalkPhaseBase::calculateBallPosition(const bool isLeftPhase)
{
  // TODO in theory we want to calculate it in every frame and calc the max x translation for the swing foot
  // The current use is just wrong in every way!
  // Also allow walking into the ball after a kick
  if(doBalanceSteps <= 0 && engine.theFrameInfo.getTimeSince(engine.theMotionRequest.ballTimeWhenLastSeen) < 1000.f)
  {
    const Pose2f scsCognition = getTransformationToZeroStep(engine.theTorsoMatrix, engine.theRobotModel, engine.theRobotDimensions, engine.theMotionRequest.odometryData, engine.theOdometryDataPreview, isLeftPhase);
    ball = scsCognition * engine.theMotionRequest.ballEstimate.position - Vector2f(std::max(step.translation.x(), 0.f), step.translation.y());
    if(!(ball.squaredNorm() > sqr(engine.clipAtBallDistance) || ball.x() <= 0.f))
      return;
  }
  ball = Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
}

void WalkPhaseBase::constructorArmCompensation(const JointAngles& jointAngles, const Angle theArmCompensation)
{
  const RobotModel model(jointAngles, engine.theRobotDimensions, engine.theMassCalibration);
  Pose3f leftSole = model.soleLeft;
  Pose3f rightSole = model.soleRight;

  Pose3f rot(Rotation::aroundY(-theArmCompensation));

  leftSole = rot * leftSole; // revert rotation by adding both rotations, and rotate translation of sole
  rightSole = rot * rightSole; // revert rotation by adding both rotations, and rotate translation of sole

  turnRL0 = (leftSole.rotation.getZAngle() - rightSole.rotation.getZAngle()) * 0.5f;

  soleRotationYL = leftSole.rotation.getYAngle();
  soleRotationXL = leftSole.rotation.getXAngle();
  forwardL0 = leftSole.translation.x() + torsoShift;
  footHL0 = engine.kinematicParameters.walkHipHeight + leftSole.translation.z();
  sideL0 = leftSole.translation.y() - this->engine.theRobotDimensions.yHipOffset;

  soleRotationYR = rightSole.rotation.getYAngle();
  soleRotationXR = rightSole.rotation.getXAngle();
  forwardR0 = rightSole.translation.x() + torsoShift;
  footHR0 = engine.kinematicParameters.walkHipHeight + rightSole.translation.z();
  sideR0 = rightSole.translation.y() + this->engine.theRobotDimensions.yHipOffset;
}

bool WalkPhaseBase::canForceFootSupportSwitch()
{
  // only force switch when walking and stepDuration is reached
  if(walkState == standing || walkState == starting || tBase < stepDuration + engine.emergencyStep.emergencyAfterStepDuration / 1000.f)
    return false;

  const float comY = (engine.theTorsoMatrix * engine.theRobotModel.centerOfMass).y();
  const float leftZ = (engine.theTorsoMatrix * engine.theRobotModel.soleLeft).translation.z();
  const float rightZ = (engine.theTorsoMatrix * engine.theRobotModel.soleRight).translation.z();
  // com shall be close to the middle AND swing foot should have not have too much height left
  if(std::abs(comY) < 15.f && (leftZ - rightZ) * (isLeftPhase ? 1.f : -1.f) < 2.f)
  {
    // the robot is standing completely still!
    if(engine.theIMUValueState.gyroValues.stableSinceTimestamp != engine.theIMUValueState.timestamp)
      return true;

    // check if gyro values are low enough
    const bool xCheck = std::abs(engine.theIMUValueState.gyroValues.deviation.x()) < engine.emergencyStep.emergencyMaxGyroDeviation && std::abs(engine.theIMUValueState.gyroValues.mean.x()) < engine.emergencyStep.emergencyMaxGyroMean;
    const bool yCheck = std::abs(engine.theIMUValueState.gyroValues.deviation.y()) < engine.emergencyStep.emergencyMaxGyroDeviation && std::abs(engine.theIMUValueState.gyroValues.mean.y()) < engine.emergencyStep.emergencyMaxGyroMean;
    // z gyro is allowed to be a lot higher, because on the real robot the problem occurs most of the time while turning
    const bool zCheck = std::abs(engine.theIMUValueState.gyroValues.deviation.z()) < engine.emergencyStep.emergencyMaxZGyroDeviation && std::abs(engine.theIMUValueState.gyroValues.mean.z()) < engine.emergencyStep.emergencyMaxGyroMean;

    return xCheck && yCheck && zCheck;
  }
  return false;
}

void WalkPhaseBase::constructorJointSpeedRegulator()
{
  JointRequest jointRequest;

  if(walkState != standing)
  {
    Pose3f leftFoot;
    Pose3f rightFoot;
    engine.calcFeetPoses(forwardL0, forwardR0, sideL0, sideR0, footHL0, footHR0, turnRL0, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFoot, rightFoot, torsoShift); // current request

    walkStepAdjustment.lastLeft = leftFoot;
    walkStepAdjustment.lastRight = rightFoot;

    static_cast<void>(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions));
  }
  else
    jointRequest.angles = engine.theJointRequest.angles;

  jointSpeedRegulator = std::make_unique<JointSpeedRegulator>(jointRequest.angles, turnRL0);
  if(walkState == standing)
  {
    currentMaxAnklePitch[Legs::left] = jointSpeedRegulator->at(Joints::lAnklePitch).max;
    currentMaxAnklePitch[Legs::right] = jointSpeedRegulator->at(Joints::rAnklePitch).max;
  }
  else
    currentMaxAnklePitch[0] = jointSpeedRegulator->at(isLeftPhase ? Joints::rAnklePitch : Joints::lAnklePitch).max;

  jointPlayOffsetRegulator = std::make_unique<JointPlayOffsetRegulator>(jointRequest);
}

JointAngles WalkPhaseBase::applyJointSpeedRegulation(JointRequest& jointRequest, const Angle supportSoleRotErrorMeasured)
{
  if(disableSpeedControl)
  {
    JointAngles angles;
    angles.angles.fill(0);
    return angles;
  }
  // Now we want to adjust the allowed speed of the joints, based on the CoM position and the rotation error between support and swing leg
  const JointRequest requestBefore = jointRequest;

  // Get com percent position for joint speed regulator
  const auto& ratioRef = engine.theRobotStableState.comInFeet[isLeftPhase ? Legs::right : Legs::left];
  const auto& ratioRefSwing = engine.theRobotStableState.comInTorso[!isLeftPhase ? Legs::right : Legs::left];

  // Apply error of measured support foot
  jointSpeedRegulator->applySoleError(supportSoleRotErrorMeasured, speedRegulatorParams, engine.theJointPlay.qualityOfRobotHardware);

  // Apply CoM error
  jointSpeedRegulator->update(speedRegulatorParams, ratioRef.forward, std::max(ratioRefSwing.outerSide, ratioRef.outerSide), isLeftPhase);

  // Apply error on joint request
  if(walkState != standing)   // only skip apply, in case the walkState is allowed to change in the future within in walkPhase
    jointSpeedRegulator->apply(jointRequest.angles, isLeftPhase);

  // Reset regulator
  jointSpeedRegulator->reset(jointRequest.angles, turnRL);

  JointAngles changes;
  for(std::size_t joint = Joints::firstLegJoint; joint < Joints::numOfJoints; joint++)
    changes.angles[joint] = jointRequest.angles[joint] - requestBefore.angles[joint];
  PLOT("module:WalkingEngine:Regulation:lHipPitch", changes.angles[Joints::lHipPitch].toDegrees());
  PLOT("module:WalkingEngine:Regulation:lKneePitch", changes.angles[Joints::lKneePitch].toDegrees());
  PLOT("module:WalkingEngine:Regulation:lAnklePitch", changes.angles[Joints::lAnklePitch].toDegrees());
  PLOT("module:WalkingEngine:Regulation:rHipPitch", changes.angles[Joints::rHipPitch].toDegrees());
  PLOT("module:WalkingEngine:Regulation:rKneePitch", changes.angles[Joints::rKneePitch].toDegrees());
  PLOT("module:WalkingEngine:Regulation:rAnklePitch", changes.angles[Joints::rAnklePitch].toDegrees());
  PLOT("module:WalkingEngine:Regulation:rHipRoll", changes.angles[Joints::rHipRoll].toDegrees());
  PLOT("module:WalkingEngine:Regulation:rAnkleRoll", changes.angles[Joints::rAnkleRoll].toDegrees());
  PLOT("module:WalkingEngine:Regulation:lHipRoll", changes.angles[Joints::lHipRoll].toDegrees());
  PLOT("module:WalkingEngine:Regulation:lAnkleRoll", changes.angles[Joints::lAnkleRoll].toDegrees());
  return changes;
}

Pose2f WalkPhaseBase::convertStep(const Pose2f& step, const Angle turnOffset)
{
  Pose2f result(step.rotation + turnOffset);
  const Vector2f hipOffset(0.f, isLeftPhase ? engine.theRobotDimensions.yHipOffset : -engine.theRobotDimensions.yHipOffset);
  const Vector2f forwardAndSide = (step.translation + hipOffset).rotated(-step.rotation * 0.5f) - 2.f * hipOffset + hipOffset.rotated(step.rotation * 0.5f);

  result.translation.x() = forwardAndSide.x();
  result.translation.y() = forwardAndSide.y();
  return result;
}

float WalkPhaseBase::temperatureShiftHandling(const Pose2f& stepSizeRequest)
{
  const unsigned char kneeTemp = std::max(engine.theJointSensorData.temperatures[Joints::lKneePitch], engine.theJointSensorData.temperatures[Joints::rKneePitch]);
  const unsigned char ankleTemp = std::max(engine.theJointSensorData.temperatures[Joints::lAnklePitch], engine.theJointSensorData.temperatures[Joints::rAnklePitch]);
  if(kneeTemp <= ankleTemp || // no adjustment needed
     engine.translationPolygonBigNotClipped.empty() || // not filled yet
     // torso shift is not possible with the requested step size
     !Geometry::isPointInsideConvexPolygon(engine.translationPolygonBigNotClipped.data(), static_cast<int>(engine.translationPolygonBigNotClipped.size()), stepSizeRequest.translation - Vector2f(engine.jointTemperatureParameters.torsoShift, 0.f)))
    return engine.kinematicParameters.torsoOffset;

  return mapToRange(static_cast<float>(kneeTemp - ankleTemp), 0.f, engine.jointTemperatureParameters.ankleKneeDiff, engine.kinematicParameters.torsoOffset, engine.kinematicParameters.torsoOffset + engine.jointTemperatureParameters.torsoShift);
}
