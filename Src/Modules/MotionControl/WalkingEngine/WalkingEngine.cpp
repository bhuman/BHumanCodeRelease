/**
 * @file WalkingEngine.cpp
 *
 * This file implements a module that generates walking motions.
 *
 * The basic ideas of using the FSRs to determine the support change and the gyroscope
 * for directly proportional feedback to the ankle joints are borrowed from the 2014
 * walking engine from rUNSWift written by Bernhard Hengst.
 *
 * @author Thomas Röfer
 * @author Philip Reichenberg
 * @author Arne Hasselbring
 */

#include "WalkingEngine.h"
#include "Platform/SystemCall.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Streaming/InStreams.h"
#include "MathBase/BHMath.h"
#include <cmath>

MAKE_MODULE(WalkingEngine);

WalkingEngine::WalkingEngine()
{
  InMapFile stream("walkingEngineCommon.cfg");
  if(stream.exists())
    stream >> static_cast<WalkingEngineCommon&>(*this);

  const DummyPhase dummy(MotionPhase::playDead);
  WalkPhase phase(*this, Pose2f(), dummy);

  translationPolygon = phase.getTranslationPolygon(configuredParameters.maxSpeedBackwards, configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.y(), true, configuredParameters.maxSideWithMaxForwardSpeed);
  translationPolygonBig = phase.getTranslationPolygon(configuredParameters.walkSpeedParams.maxSpeedBackwards, configuredParameters.walkSpeedParams.maxSpeed.translation.x(), configuredParameters.walkSpeedParams.maxSpeed.translation.y(), true, std::optional<float>());

  std::vector<Vector2f> translationPolygonTemp = phase.getTranslationPolygon(configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.y(), true, std::optional<float>());
  filterTranslationPolygon(translationPolygonAfterKick, translationPolygonTemp, translationPolygonTemp);

  translationPolygonBigNotClipped = phase.getTranslationPolygon(walkSpeedParamsWalkStep.maxSpeedBackwards, walkSpeedParamsWalkStep.maxSpeed.translation.x(), walkSpeedParamsWalkStep.maxSpeed.translation.y(), false, std::optional<float>());

  // Ensure the values set are in a somewhat reasonable range (commented out for T1)
  //ASSERT(kinematicParameters.walkHipHeight.min > 200.f && kinematicParameters.walkHipHeight.min < 245.f);
  //ASSERT(kinematicParameters.walkHipHeight.max > 200.f && kinematicParameters.walkHipHeight.max < 245.f);
  ASSERT(kinematicParameters.walkHipHeight.max >= kinematicParameters.walkHipHeight.min);
}

void WalkingEngine::reg()
{
  PUBLISH(reg);
  REG_CLASS_WITH_BASE(WalkingEngine, WalkingEngineCommon);
}

void WalkingEngine::update(WalkStepData& walkStepData)
{
  walkStepData.updateCounter = [&walkStepData](const bool predictedStep)
  {
    walkStepData.usedPredictedSwitch += predictedStep ? 1 : 0;
  };
  walkStepData.updateWalkValues = [this, &walkStepData](const Pose2f& stepTarget, const float stepDuration, const bool isLeftPhase)
  {
    walkStepData.isLeftPhase = isLeftPhase;
    walkStepData.stepTarget = stepTarget;
    walkStepData.stepDuration = stepDuration;
    walkStepData.lastUpdate = theFrameInfo.time;
  };

  walkStepData.yHipOffset = theRobotDimensions.yHipOffset + kinematicParameters.yHipOffset;
}

void WalkingEngine::update(WalkGenerator& walkGenerator)
{
  MODIFY("module:WalkingEngine:common", static_cast<WalkingEngineCommon&>(*this));
  DECLARE_DEBUG_DRAWING3D("module:WalkStepAdjustment:balance", "robot");
  DECLARE_DEBUG_DRAWING3D("module:WalkDelayPhase:hitPoint", "robot");
  DECLARE_PLOT("module:WalkingEngine:current:left:x");
  DECLARE_PLOT("module:WalkingEngine:request:left:x");
  DECLARE_PLOT("module:WalkingEngine:current:left:y");
  DECLARE_PLOT("module:WalkingEngine:request:left:y");
  DECLARE_PLOT("module:WalkingEngine:current:left:z");
  DECLARE_PLOT("module:WalkingEngine:request:left:z");
  DECLARE_PLOT("module:WalkingEngine:current:right:x");
  DECLARE_PLOT("module:WalkingEngine:request:right:x");
  DECLARE_PLOT("module:WalkingEngine:current:right:y");
  DECLARE_PLOT("module:WalkingEngine:current:right:y");
  DECLARE_PLOT("module:WalkingEngine:request:right:y");
  DECLARE_PLOT("module:WalkingEngine:current:right:z");
  DECLARE_PLOT("module:WalkingEngine:request:right:z");
  DECLARE_PLOT("module:WalkingEngine:current:left:rot:x");
  DECLARE_PLOT("module:WalkingEngine:request:left:rot:x");
  DECLARE_PLOT("module:WalkingEngine:current:left:rot:y");
  DECLARE_PLOT("module:WalkingEngine:pred:left:rot:y");
  DECLARE_PLOT("module:WalkingEngine:request:left:rot:y");
  DECLARE_PLOT("module:WalkingEngine:current:left:rot:z");
  DECLARE_PLOT("module:WalkingEngine:request:left:rot:z");
  DECLARE_PLOT("module:WalkingEngine:current:right:rot:x");
  DECLARE_PLOT("module:WalkingEngine:request:right:rot:x");
  DECLARE_PLOT("module:WalkingEngine:current:right:rot:y");
  DECLARE_PLOT("module:WalkingEngine:request:right:rot:y");
  DECLARE_PLOT("module:WalkingEngine:pred:right:rot:y");
  DECLARE_PLOT("module:WalkingEngine:current:right:rot:z");
  DECLARE_PLOT("module:WalkingEngine:request:right:rot:z");
  DECLARE_PLOT("module:WalkingEngine:armCompensation");
  DECLARE_PLOT("module:WalkingEngine:Data:leftAdjustment");
  DECLARE_PLOT("module:WalkingEngine:Data:rightAdjustment");
  DECLARE_PLOT("module:WalkStepAdjustment:Data:leftAdjustment");
  DECLARE_PLOT("module:WalkStepAdjustment:Data:rightAdjustment");
  DECLARE_PLOT("module:WalkingEngine:SpeedPolygon:front");
  DECLARE_PLOT("module:WalkingEngine:SpeedPolygon:back");
  DECLARE_PLOT("module:WalkingEngine:Data:sideBalance");
  DECLARE_PLOT("module:WalkingEngine:Data:refCOMY");
  DECLARE_PLOT("module:WalkingEngine:Gyro:sagittal");
  DECLARE_PLOT("module:WalkingEngine:Gyro:lateral");
  DECLARE_PLOT("module:WalkingEngine:Gyro:hipKnee");
  DECLARE_PLOT("module:WalkingEngine:Gyro:hipRoll");
  DECLARE_PLOT("module:WalkingEngine:Data:supportSoleRotationY");
  DECLARE_PLOT("module:WalkingEngine:Data:supportSoleRotationCompensationX");
  DECLARE_PLOT("module:WalkingEngine:Data:supportSoleRotationCompensationY");
  DECLARE_PLOT("module:WalkingEngine:Data:feetHeightDifference");
  DECLARE_PLOT("module:WalkingEngine:Regulation:lHipPitch");
  DECLARE_PLOT("module:WalkingEngine:Regulation:lKneePitch");
  DECLARE_PLOT("module:WalkingEngine:Regulation:lAnklePitch");
  DECLARE_PLOT("module:WalkingEngine:Regulation:lHipRoll");
  DECLARE_PLOT("module:WalkingEngine:Regulation:lAnkleRoll");
  DECLARE_PLOT("module:WalkingEngine:Regulation:rHipPitch");
  DECLARE_PLOT("module:WalkingEngine:Regulation:rKneePitch");
  DECLARE_PLOT("module:WalkingEngine:Regulation:rAnklePitch");
  DECLARE_PLOT("module:WalkingEngine:Regulation:rHipRoll");
  DECLARE_PLOT("module:WalkingEngine:Regulation:rAnkleRoll");

  DECLARE_PLOT("module:WalkingEngine:JointOffset:lHipPitch");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:lKneePitch");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:lAnklePitch");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:lHipRoll");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:lAnkleRoll");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:rHipPitch");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:rKneePitch");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:rAnklePitch");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:rHipRoll");
  DECLARE_PLOT("module:WalkingEngine:JointOffset:rAnkleRoll");
  DECLARE_DEBUG_RESPONSE("module:WalkingEngine:feetPositions");

  DECLARE_PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:delta");
  DECLARE_PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:torso");
  DECLARE_PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:gyro");
  DECLARE_PLOT("module:WalkingEngine:WalkStepAdjustment:scaling:sum");

  DEBUG_RESPONSE("module:WalkingEngine:feetPositions")
  {
    RobotModel requestedModel(theJointRequest, theRobotDimensions, theMassCalibration);
    Pose3f& left = requestedModel.soleLeft;
    Pose3f& right = requestedModel.soleRight;
    PLOT("module:WalkingEngine:current:left:x", theRobotModel.soleLeft.translation.x());
    PLOT("module:WalkingEngine:request:left:x", left.translation.x());
    PLOT("module:WalkingEngine:current:left:y", theRobotModel.soleLeft.translation.y());
    PLOT("module:WalkingEngine:request:left:y", left.translation.y());
    PLOT("module:WalkingEngine:current:left:z", theRobotModel.soleLeft.translation.z());
    PLOT("module:WalkingEngine:request:left:z", left.translation.z());
    PLOT("module:WalkingEngine:current:right:x", theRobotModel.soleRight.translation.x());
    PLOT("module:WalkingEngine:request:right:x", right.translation.x());
    PLOT("module:WalkingEngine:current:right:y", theRobotModel.soleRight.translation.y());
    PLOT("module:WalkingEngine:request:right:y", right.translation.y());
    PLOT("module:WalkingEngine:current:right:z", theRobotModel.soleRight.translation.z());
    PLOT("module:WalkingEngine:request:right:z", right.translation.z());

    PLOT("module:WalkingEngine:current:left:rot:x", Angle(theRobotModel.soleLeft.rotation.getXAngle()).toDegrees());
    PLOT("module:WalkingEngine:request:left:rot:x", Angle(left.rotation.getXAngle()).toDegrees());
    PLOT("module:WalkingEngine:current:left:rot:y", Angle(theRobotModel.soleLeft.rotation.getYAngle()).toDegrees());
    PLOT("module:WalkingEngine:request:left:rot:y", Angle(left.rotation.getYAngle()).toDegrees());
    PLOT("module:WalkingEngine:current:left:rot:z", Angle(theRobotModel.soleLeft.rotation.getZAngle()).toDegrees());
    PLOT("module:WalkingEngine:request:left:rot:z", Angle(left.rotation.getZAngle()).toDegrees());
    PLOT("module:WalkingEngine:current:right:rot:x", Angle(theRobotModel.soleRight.rotation.getXAngle()).toDegrees());
    PLOT("module:WalkingEngine:request:right:rot:x", Angle(right.rotation.getXAngle()).toDegrees());
    PLOT("module:WalkingEngine:current:right:rot:y", Angle(theRobotModel.soleRight.rotation.getYAngle()).toDegrees());
    PLOT("module:WalkingEngine:request:right:rot:y", Angle(right.rotation.getYAngle()).toDegrees());
    PLOT("module:WalkingEngine:current:right:rot:z", Angle(theRobotModel.soleRight.rotation.getZAngle()).toDegrees());
    PLOT("module:WalkingEngine:request:right:rot:z", Angle(right.rotation.getZAngle()).toDegrees());

    JointRequest other = theJointRequest;
    other.angles = theJointAngles.angles;
    // Update the joints that are predicted.
    if(theJointAnglePred.isValid)
    {
      FOREACH_ENUM(Joints::Joint, joint)
      {
        if(theJointAnglePred.angles[joint] != SensorData::ignore)
          other.angles[joint] = theJointAnglePred.angles[joint];
      }
    }
    const RobotModel requestedModelOther(other, theRobotDimensions, theMassCalibration);
    left = requestedModelOther.soleLeft;
    right = requestedModelOther.soleRight;

    PLOT("module:WalkingEngine:pred:left:rot:y", Angle(left.rotation.getYAngle()).toDegrees());
    PLOT("module:WalkingEngine:pred:right:rot:y", Angle(right.rotation.getYAngle()).toDegrees());
  }

  DEBUG_RESPONSE_ONCE("module:WalkingEngine:init:parameters")
  {
    const DummyPhase dummy(MotionPhase::playDead);
    WalkPhase phase(*this, Pose2f(), dummy);

    translationPolygon = phase.getTranslationPolygon(configuredParameters.maxSpeedBackwards, configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.y(), true, configuredParameters.maxSideWithMaxForwardSpeed);
    std::vector<Vector2f> translationPolygonTemp = phase.getTranslationPolygon(configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.y(), true, std::optional<float>());

    filterTranslationPolygon(translationPolygonAfterKick, translationPolygonTemp, translationPolygonTemp);
    translationPolygonBigNotClipped = phase.getTranslationPolygon(walkSpeedParamsWalkStep.maxSpeedBackwards, walkSpeedParamsWalkStep.maxSpeed.translation.x(), walkSpeedParamsWalkStep.maxSpeed.translation.y(), false, std::optional<float>());
  }

  filteredGyroX = balanceParameters.gyroLowPassRatio * filteredGyroX + (1.f - balanceParameters.gyroLowPassRatio) * theInertialData.gyro.x();
  filteredGyroY = balanceParameters.gyroLowPassRatio * filteredGyroY + (1.f - balanceParameters.gyroLowPassRatio) * theInertialData.gyro.y();

  // Set walk functions
  walkGenerator.createPhase = [this](const Pose2f& step, const MotionPhase& lastPhase, float delay)->std::unique_ptr<MotionPhase>
  {
    Pose2f useStepTarget = step; // SystemCall::getMode() == SystemCall::logFileReplay ? theWalkStepData.stepTarget : step;
    if((lastPhase.type == MotionPhase::kick || lastPhase.type == MotionPhase::freeze) && useStepTarget == Pose2f())
      useStepTarget.translation.x() = 0.01f;

    // If active, no delay phase
    if(walkHipShiftPhaseParameters.useHipShift && lastPhase.type != MotionPhase::playDead)
      return std::make_unique<WalkHipShiftPhase>(*this, useStepTarget, lastPhase);

    // Determine delay duration and height scaling
    float delayPercentHeight = step == Pose2f() ? 0.5f : 1.f;
    if(lastPhase.type == MotionPhase::kick)
    {
      theRobotStableState.predictRotation(theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget), false, false);
      const bool comNearSwingFoot = (theKickGenerator.wasLeftPhase(lastPhase) && theRobotStableState.comInFeet[Legs::left].outerSide != 0.f) ||
      (!theKickGenerator.wasLeftPhase(lastPhase) && theRobotStableState.comInFeet[Legs::right].outerSide != 0.f);

      if(!comNearSwingFoot &&
         (theKickGenerator.wasLeftPhase(lastPhase) ? theSolePressureState.legInfo[Legs::left] : theSolePressureState.legInfo[Legs::right]).hasPressure)
        delay = sideStabilizeParameters.delayRange.max;
    }

    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhaseDummy = static_cast<const WalkPhaseBase&>(lastPhase);
      const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget);
      theRobotStableState.predictRotation(isLeftPhase, lastWalkPhaseDummy.walkKickStep.currentKick == WalkKicks::none, true);
      Legs::Leg leg = isLeftPhase ? Legs::left : Legs::right;

      const SideStabilizeParameters& p = sideStabilizeParameters;
      const Angle turnRL0 = (theRobotModel.soleLeft.rotation.getZAngle() - theRobotModel.soleRight.rotation.getZAngle()) * 0.5f;
      const float offset = mapToRange(static_cast<float>(turnRL0), static_cast<float>(p.turnIncreaseRange.min), static_cast<float>(p.turnIncreaseRange.max), 0.f, p.increaseThreshold);
      const bool stopping = (useStepTarget.translation.x() == 0.f && useStepTarget.translation.y() == 0.f && useStepTarget.rotation == 0_deg) || !theGroundContactState.contact;
      const bool handleBasedOnCom = theRobotStableState.comInFeet[leg].outerSide > p.minOuterSide + offset && theRobotStableState.getTurnPoint(leg).outerSide > p.minOuterSideStop + offset;
      const bool handleBasedOnStepDuration = std::abs(useStepTarget.translation.y()) < p.maxNextSideStepSize && std::abs(lastWalkPhaseDummy.sideHipShift) >= p.maxSideHipShift * p.minHipShiftRatioForDelay && lastWalkPhaseDummy.lastTBase > p.lastLastWalkPhaseStepDuration && lastWalkPhaseDummy.tBase < p.lastWalkPhaseStepDuration;
      if(!stopping && (handleBasedOnCom || handleBasedOnStepDuration))
      {
        delay = p.delayRange.min;
        delayPercentHeight = mapToRange(theRobotStableState.comInFeet[leg].outerSide, p.comInOuterInterpolationRange.min, p.comInOuterInterpolationRange.max, p.heightRange.min, p.heightRange.max);
      }
    }
    if(lastPhase.type == MotionPhase::stand && step != Pose2f())
    {
      delay = sideStabilizeParameters.delayRange.min;
      delayPercentHeight = sideStabilizeParameters.firstStepHeightRange;
    }

    // Return next walk phase
    if(delay <= 0.f)
      return std::make_unique<WalkPhase>(*this, useStepTarget, lastPhase);
    else
      return std::make_unique<WalkDelayPhase>(*this, useStepTarget, lastPhase, delay, delayPercentHeight);
  };

  walkGenerator.createPhaseWithNextPhase = [this](const WalkKickStep& walkKickStep, const MotionPhase& lastPhase, const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback, float delay)->std::unique_ptr<MotionPhase>
  {
    Pose2f useStepTarget = SystemCall::getMode() == SystemCall::logFileReplay ? theWalkStepData.stepTarget : walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTarget;
    if((lastPhase.type == MotionPhase::kick || lastPhase.type == MotionPhase::freeze) && useStepTarget == Pose2f())
      useStepTarget.translation.x() = 0.01f;

    // If active, no delay phase
    if(walkHipShiftPhaseParameters.useHipShift)
      return std::make_unique<WalkHipShiftPhase>(*this, useStepTarget, lastPhase, createNextPhaseCallback, walkKickStep);

    // Determine delay duration and height scaling, if no kick was set
    float delayPercentHeight = delay != 0.f ? 0.5f : 1.f;
    if(delay == 0.f && lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhaseDummy = static_cast<const WalkPhaseBase&>(lastPhase);
      const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget);
      theRobotStableState.predictRotation(isLeftPhase, lastWalkPhaseDummy.walkKickStep.currentKick == WalkKicks::none, true);
      Legs::Leg leg = isLeftPhase ? Legs::left : Legs::right;

      const SideStabilizeParameters& p = sideStabilizeParameters;
      const Angle turnRL0 = (theRobotModel.soleLeft.rotation.getZAngle() - theRobotModel.soleRight.rotation.getZAngle()) * 0.5f;
      const float offset = mapToRange(static_cast<float>(turnRL0), static_cast<float>(p.turnIncreaseRange.min), static_cast<float>(p.turnIncreaseRange.max), 0.f, p.increaseThreshold);
      const bool stopping = (useStepTarget.translation.x() == 0.f && useStepTarget.translation.y() == 0.f && useStepTarget.rotation == 0_deg) || !theGroundContactState.contact;
      if(!stopping && theRobotStableState.comInFeet[leg].outerSide > p.minOuterSide + offset && theRobotStableState.getTurnPoint(leg).outerSide > p.minOuterSide + offset)
      {
        delay = motionCycleTime * 3.f;
        delayPercentHeight = mapToRange(theRobotStableState.comInFeet[leg].outerSide, p.comInOuterInterpolationRange.min, p.comInOuterInterpolationRange.max, p.heightRange.min, p.heightRange.max);
      }
    }

    // Return next walk kick phase
    ASSERT(walkKickStep.keyframe.size() > 0);
    if(delay <= 0.f)
      return std::make_unique<WalkPhase>(*this, useStepTarget, lastPhase, false, createNextPhaseCallback, walkKickStep);
    else
      return std::make_unique<WalkDelayPhase>(*this, useStepTarget, lastPhase, delay, delayPercentHeight, createNextPhaseCallback, walkKickStep);
  };

  walkGenerator.isNextLeftPhase = [this](const MotionPhase& lastPhase, const Pose2f& stepTarget)
  {
    switch(lastPhase.type)
    {
      case MotionPhase::walk:
      {
        const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
        if(!lastWalkPhase.supportSwitchInfo.isFootSupportSwitch)
          return theFootSupport.support > 0.f;
        return theFootSupport.support < 0.f;
      }
      case MotionPhase::kick:
        return theKickGenerator.wasLeftPhase(lastPhase);
      case MotionPhase::stand:
      {
        const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
        if(lastWalkPhase.isDelay != 0.f) // if the stand is actually a delay phase, we want to return the planned leftPhase
          return lastWalkPhase.isLeftPhase;
        return stepTarget.translation.y() != 0.f // first step based on side translation
               ? stepTarget.translation.y() > 0
               : (stepTarget.rotation != 0_deg ? stepTarget.rotation > 0_deg // else first step based rotation
                  : theFootSupport.support < 0.f); // otherwise based on support foot
      }
      default:
        return theFootSupport.support < 0.f;
    }
  };

  walkGenerator.wasLastPhaseLeftPhase = [this](const MotionPhase& lastPhase)
  {
    if(lastPhase.type != MotionPhase::walk)
      return theFootSupport.support > 0.f;

    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    return lastWalkPhase.isLeftPhase;
  };

  walkGenerator.getLastWalkPhase = [](WalkKickStep& walkKickStep, Pose2f& step, const MotionPhase& lastPhase)
  {
    if(lastPhase.type != MotionPhase::walk)
    {
      step = Pose2f();
      walkKickStep = WalkKickStep();
      return;
    }
    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    step = lastWalkPhase.step;
    walkKickStep = lastWalkPhase.walkKickStep;
  };

  walkGenerator.isWalkDelayPossible = [this](const MotionPhase& lastPhase, const float delay, const Pose2f& stepTarget, const bool isKick)
  {
    if(lastPhase.type != MotionPhase::walk)
      return true; // Could be a bad idea, but currently there should be no case where it is a bad idea

    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, stepTarget);
    return WalkDelayPhase::isWalkDelayPossible(lastWalkPhase.forwardL, lastWalkPhase.forwardR, lastWalkPhase.sideL, lastWalkPhase.sideR,
                                               lastWalkPhase.footHL, lastWalkPhase.footHR, lastWalkPhase.turnRL, lastWalkPhase.currentWalkHipHeight,
                                               lastWalkPhase.soleRotationYL, lastWalkPhase.soleRotationXL,
                                               lastWalkPhase.soleRotationYR, lastWalkPhase.soleRotationXR,
                                               isLeftPhase, false, delay,
                                               walkDelayParameters, *this, isKick);
  };

  walkGenerator.getRotationRange = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio)
  {
    const float useRobotStateFactor = useJointPlayScaling ? theJointPlay.qualityOfRobotHardware : 1.f;
    const Angle rotation = configuredParameters.maxSpeed.rotation * useRobotStateFactor + configuredParameters.minSpeed.rotation * (1.f - useRobotStateFactor);
    const float innerTurn = 2.f * stepSizeParameters.insideTurnRatio * rotation * std::abs(walkSpeedRatio.rotation) * kinematicParameters.baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - stepSizeParameters.insideTurnRatio) * rotation * std::abs(walkSpeedRatio.rotation) * kinematicParameters.baseWalkPeriod / 1000.f;
    return Rangea(isLeftPhase ? -innerTurn : -outerTurn, isLeftPhase ? outerTurn : innerTurn);
  };

  walkGenerator.getStepRotationRangeOther = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio, Vector2f step,
                                                   const bool isFastWalk, const std::vector<Vector2f>& translationPolygon,
                                                   const bool ignoreXTranslation, const bool isMaxPossibleStepSize)
  {
    ASSERT(translationPolygon.size() >= 4);
    if(translationPolygon.size() < 4)
      return Rangea(0_deg, 0_deg);

    const float durationValue = kinematicParameters.baseWalkPeriod / 1000.f;
    const float defaultMaxSide = 2.f * configuredParameters.maxSpeed.translation.y() * durationValue;

    if(!isMaxPossibleStepSize)
    {
      if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), step))
      {
        Vector2f p1;
        VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f),
                                                               step / step.norm()), p1, false));
        step = p1;
      }
    }

    Vector2f stepRatio(0.f, 0.f);
    Rangef xRange(0.1f, 0.1f);
    Rangef yRange(0.1f, 0.1f);
    for(const Vector2f& p : translationPolygon)
    {
      xRange.min = std::min(xRange.min, p.x());
      xRange.max = std::max(xRange.max, p.x());
      yRange.min = std::min(yRange.min, p.y());
      yRange.max = std::max(yRange.max, p.y());
      if(isMaxPossibleStepSize)
      {
        yRange.min = std::max(yRange.min, -defaultMaxSide);
        yRange.max = std::min(yRange.max, defaultMaxSide);
      }
    }

    if(!ignoreXTranslation)
    {
      if(step.x() < 0)
        stepRatio.x() = step.x() / xRange.min;
      else if(step.x() > 0.f)
        stepRatio.x() = step.x() / xRange.max;
    }
    if(step.y() < 0)
      stepRatio.y() = step.y() / yRange.min;
    else if(step.y() > 0.f)
      stepRatio.y() = step.y() / yRange.max;

    stepRatio.x() = Rangef::ZeroOneRange().limit(stepRatio.x());
    stepRatio.y() = Rangef::ZeroOneRange().limit(stepRatio.y());

    // Ensure stability
    if(isMaxPossibleStepSize)
    {
      stepRatio.y() = std::sqrt(stepRatio.y());
      stepRatio.x() = std::sqrt(stepRatio.x());
    }

    const Rangea maxAllowedRotation = getMaxRotationToStepFactor(isFastWalk, isLeftPhase, 0, stepRatio);

    const float useRobotStateFactor = useJointPlayScaling ? theJointPlay.qualityOfRobotHardware : 1.f;
    const Angle rotation = configuredParameters.maxSpeed.rotation * useRobotStateFactor + configuredParameters.minSpeed.rotation * (1.f - useRobotStateFactor);
    const float innerTurn = 2.f * stepSizeParameters.insideTurnRatio * rotation * std::abs(walkSpeedRatio.rotation) * kinematicParameters.baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - stepSizeParameters.insideTurnRatio) * rotation * std::abs(walkSpeedRatio.rotation) * kinematicParameters.baseWalkPeriod / 1000.f;
    return Rangea(isLeftPhase ? maxAllowedRotation.limit(-innerTurn) : maxAllowedRotation.limit(-outerTurn), isLeftPhase ? maxAllowedRotation.limit(outerTurn) : maxAllowedRotation.limit(innerTurn));
  };

  walkGenerator.getStepRotationRange = [&walkGenerator](const bool isLeftPhase, const Pose2f& walkSpeedRatio, Vector2f step,
                                                        const bool isFastWalk, const MotionPhase& lastPhase, const bool ignoreXTranslation, const bool clipTranslation)
  {
    std::vector<Vector2f> translationPolygon;
    std::vector<Vector2f> translationPolygonNoCenter;
    walkGenerator.getTranslationPolygon(isLeftPhase, 0, lastPhase, walkSpeedRatio, translationPolygon, translationPolygonNoCenter, isFastWalk, false);

    return walkGenerator.getStepRotationRangeOther(isLeftPhase, walkSpeedRatio, step, isFastWalk, translationPolygon, ignoreXTranslation, clipTranslation);
  };

  walkGenerator.getTranslationPolygon = [this](const bool isLeftPhase, float rotation, const MotionPhase& lastPhase, const Pose2f& walkSpeedRatio, std::vector<Vector2f>& translationPolygon, std::vector<Vector2f>& translationPolygonNoCenter, const bool fastWalk, const bool useMaxPossibleStepSize)
  {
    bool useFastWalk = fastWalk;
    // After an InWalkKick, the next steps are balance steps to ensure that the robot will not fall
    Vector2f forwardBalance = Vector2f(0.f, 0.f);
    Pose2f useWalkSpeedRatio = walkSpeedRatio;
    bool armsAreBack = false;
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      if(lastWalkPhase.walkStepAdjustment.reduceWalkingSpeed > 0)
        useWalkSpeedRatio.translation.x() *= commonSpeedParameters.walkSpeedReductionFactor;
      if(lastWalkPhase.noFastTranslationPolygonSteps > 0)
        useFastWalk = false;

      armsAreBack = theFrameInfo.getTimeSince(lastWalkPhase.leftArmInterpolationStart) < motionCycleTime * 2.f * 1000.f || theFrameInfo.getTimeSince(lastWalkPhase.leftArmInterpolationStart) < motionCycleTime * 2.f * 1000.f;
    }

    Vector2f frontLeft, backRight;
    const Pose2f lastStep = lastPhase.type == MotionPhase::walk ? static_cast<const WalkPhase&>(lastPhase).step : Pose2f();
    const Vector2f maxStepSizeChange = commonSpeedParameters.maxAcceleration * kinematicParameters.baseWalkPeriod / 1000.f;
    const Vector2f maxStepSizeChangeToZero = commonSpeedParameters.maxDeceleration * kinematicParameters.baseWalkPeriod / 1000.f;
    if(lastStep.translation.x() > 0.f)
    {
      frontLeft.x() = lastStep.translation.x() + maxStepSizeChange.x();
      backRight.x() = std::max(lastStep.translation.x() - maxStepSizeChangeToZero.x(), 0.f);
    }
    else if(lastStep.translation.x() < 0.f)
    {
      frontLeft.x() = std::min(lastStep.translation.x() + maxStepSizeChangeToZero.x(), 0.f);
      backRight.x() = lastStep.translation.x() - maxStepSizeChange.x();
    }
    else
    {
      frontLeft.x() = maxStepSizeChange.x();
      backRight.x() = -maxStepSizeChange.x(); // when walking circular around the ball, the feet must be allowed to move far backward
    }

    backRight.y() = -1000.f;
    frontLeft.y() = 1000.f;

    // Scale by JointPlay
    const float useRobotStateFactor = useJointPlayScaling ? theJointPlay.qualityOfRobotHardware : 1.f;
    Vector2f useSpeed = useMaxPossibleStepSize ? configuredParameters.walkSpeedParams.maxSpeed.translation : configuredParameters.maxSpeed.translation * useRobotStateFactor + configuredParameters.minSpeed.translation * (1.f - useRobotStateFactor);
    const float useSpeedBackwards = useMaxPossibleStepSize ? configuredParameters.walkSpeedParams.maxSpeedBackwards : configuredParameters.maxSpeedBackwards * useRobotStateFactor + configuredParameters.minSpeedBackwards * (1.f - useRobotStateFactor);

    // Limit to maximum speed (which is influenced by the rotation).
    // When the arms are on the back, the robot will unintentionally turn more in each step.
    // Allow more rotation + translation, so the robot does not walk that much slower
    const Vector2f stepSizeFactor = getStepSizeFactor(rotation, useFastWalk, isLeftPhase, armsAreBack ? 10_deg : 0_deg, useWalkSpeedRatio.translation);

    backRight.x() = std::max(backRight.x(), stepSizeFactor.x() * -useSpeedBackwards * kinematicParameters.baseWalkPeriod / 1000.f);
    backRight.y() = std::max(backRight.y(), stepSizeFactor.y() * -2.f * useSpeed.y() * kinematicParameters.baseWalkPeriod / 1000.f);
    frontLeft.x() = std::min(frontLeft.x(), stepSizeFactor.x() * useSpeed.x() * kinematicParameters.baseWalkPeriod / 1000.f);
    frontLeft.y() = std::min(frontLeft.y(), stepSizeFactor.y() * 2.f * useSpeed.y() * kinematicParameters.baseWalkPeriod / 1000.f);

    if(useFastWalk)
    {
      const float useMinXBackwardTranslation = stepSizeParameters.minXBackwardTranslationFastRange.min * (1.f - useRobotStateFactor) + stepSizeParameters.minXBackwardTranslationFastRange.max * useRobotStateFactor;
      frontLeft.x() = std::max(stepSizeParameters.minXForwardTranslationFast * useWalkSpeedRatio.translation.x(), frontLeft.x());
      backRight.x() = std::min(useMinXBackwardTranslation * useWalkSpeedRatio.translation.x(), backRight.x());
    }

    // Step size in x translation has a min size
    const float maxMinStepX = std::min(stepSizeParameters.minXTranslationStep, (useWalkSpeedRatio.translation.x() >= 0.f ? useSpeed.x() : useSpeedBackwards) * kinematicParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()) + 0.01f);

    backRight.x() = std::min(backRight.x(), -maxMinStepX);
    frontLeft.x() = std::max(frontLeft.x(), maxMinStepX);

    // (0,0) must be part of the rectangle.
    backRight.x() = std::min(backRight.x(), -.01f);
    frontLeft.x() = std::max(frontLeft.x(), .01f);
    backRight.y() = std::min(backRight.y(), -.01f);
    frontLeft.y() = std::max(frontLeft.y(), .01f);

    Vector2f frontLeftNoCenter = frontLeft;
    Vector2f backRightNoCenter = backRight;

    generateTranslationPolygon(translationPolygon, backRight, frontLeft, useMaxPossibleStepSize);
    generateTranslationPolygon(translationPolygonNoCenter, backRightNoCenter, frontLeftNoCenter, useMaxPossibleStepSize);

    ASSERT(translationPolygon.size() != 0);
    ASSERT(translationPolygonNoCenter.size() != 0);
  };

  walkGenerator.generateTranslationPolygon = [this](const bool isLeftPhase,  const Angle rotation, const Pose2f& walkSpeedRatio,
                                                    std::vector<Vector2f>& translationPolygon, Vector2f backRight, Vector2f frontLeft,
                                                    const bool useFastWalk, const bool useMaxPossibleStepSize)
  {
    const Vector2f stepSizeFactor = getStepSizeFactor(rotation, useFastWalk, isLeftPhase, 0, walkSpeedRatio.translation);

    // (0,0) must be part of the rectangle.
    const float maxXRatio = std::min(stepSizeFactor.x(), std::max(std::abs(walkSpeedRatio.translation.x()), 0.01f));
    const float maxYRatio = std::min(stepSizeFactor.y(), std::max(std::abs(walkSpeedRatio.translation.y()), 0.01f));
    backRight.x() = std::min(backRight.x() * maxXRatio, -.01f);
    frontLeft.x() = std::max(frontLeft.x() * maxXRatio, .01f);
    backRight.y() = std::min(backRight.y() * maxYRatio, -.01f);
    frontLeft.y() = std::max(frontLeft.y() * maxYRatio, .01f);
    generateTranslationPolygon(translationPolygon, backRight, frontLeft, useMaxPossibleStepSize);
  };

  walkGenerator.getStartOffsetOfNextWalkPhase = [this](const MotionPhase& lastPhase)
  {
    Pose2f left;
    Pose2f right;
    Pose2f lastStep;
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      left = Pose2f(lastWalkPhase.turnRL, lastWalkPhase.forwardL, lastWalkPhase.lastPrevSideL);
      right = Pose2f(-lastWalkPhase.turnRL, lastWalkPhase.forwardR, lastWalkPhase.lastPrevSideR);
      lastStep = lastWalkPhase.step;
    }
    else
    {
      const RobotModel lastRobotModel(theJointRequest, theRobotDimensions, theMassCalibration);
      left.translate(lastRobotModel.soleLeft.translation.head<2>() + Vector2f(kinematicParameters.torsoOffset, -theRobotDimensions.yHipOffset - kinematicParameters.yHipOffset)).rotate(lastRobotModel.soleLeft.rotation.getZAngle());
      right.translate(lastRobotModel.soleRight.translation.head<2>() + Vector2f(kinematicParameters.torsoOffset, theRobotDimensions.yHipOffset + kinematicParameters.yHipOffset)).rotate(lastRobotModel.soleRight.rotation.getZAngle());
    }
    return std::make_tuple(left, right, lastStep);
  };

  walkGenerator.getLastStepChange = [](const MotionPhase& lastPhase)
  {
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      Pose2f stepChange;
      stepChange.rotation = lastWalkPhase.turnRL0 - lastWalkPhase.turnRL;
      const float swingSign = lastWalkPhase.isLeftPhase ? 1.f : -1.f;
      stepChange.translation.x() = (swingSign * (lastWalkPhase.forwardL - lastWalkPhase.forwardL0) - swingSign * (lastWalkPhase.forwardR - lastWalkPhase.forwardR0)) * 0.5f;
      stepChange.translation.y() = swingSign * (lastWalkPhase.sideL - lastWalkPhase.sideL0) - swingSign * (lastWalkPhase.sideR - lastWalkPhase.sideR0);
      return stepChange;
    }
    return Pose2f();
  };

  walkGenerator.getLastStepChangeWithOffsets = [&](const MotionPhase& lastPhase)
  {
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      const RobotModel model(theJointAngles, theRobotDimensions, theMassCalibration);
      Pose3f leftSole = model.soleLeft;
      Pose3f rightSole = model.soleRight;

      Pose3f rot(Rotation::aroundY(-lastWalkPhase.armCompensationTilt[0]));

      leftSole = rot * leftSole; // revert rotation by adding both rotations, and rotate translation of sole
      rightSole = rot * rightSole; // revert rotation by adding both rotations, and rotate translation of sole

      const Angle turn = (leftSole.rotation.getZAngle() - rightSole.rotation.getZAngle()) * 0.5f;

      const float forwardL = leftSole.translation.x() + kinematicParameters.torsoOffset;
      const float sideL = leftSole.translation.y() - theRobotDimensions.yHipOffset - kinematicParameters.yHipOffset;

      const float forwardR = rightSole.translation.x() + kinematicParameters.torsoOffset;
      const float sideR = rightSole.translation.y() + theRobotDimensions.yHipOffset + kinematicParameters.yHipOffset;

      Pose2f swing;
      Pose2f support;
      Pose2f step;
      const Pose2f left(turn, forwardL, std::max(0.f, sideL));
      const Pose2f right(turn, forwardR, std::min(0.f, sideR));

      return std::make_tuple(left, right, Pose2f(turn, lastWalkPhase.isLeftPhase ? (forwardL - forwardR) : (forwardR - forwardL), lastWalkPhase.isLeftPhase ? (left.translation.y() - right.translation.y()) : (right.translation.y() - left.translation.y())));
    }
    return std::make_tuple(Pose2f(), Pose2f(), Pose2f());
  };

  walkGenerator.wasLastPhaseInWalkKick = [](const MotionPhase& lastPhase)
  {
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      return lastWalkPhase.walkKickStep.currentKick != WalkKicks::none;
    }
    return false;
  };
}

Vector2f WalkingEngine::getStepSizeFactor(const Angle rotation, const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& walkSpeedRatio)
{
  const Angle useReduceFactor = std::max(customReduceOffset, stepSizeParameters.reduceTranslationFromRotation.x());
  const float tFactorX = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceFactor) / (stepSizeParameters.noTranslationFromRotation.x() - useReduceFactor))));
  const float reduceTransitionY = !isFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : ((isLeftPhase && rotation > 0_deg) || (!isLeftPhase && rotation < 0_deg) ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
  const float useReduceTranslationFromRotation = !isFastWalk ? stepSizeParameters.reduceTranslationFromRotation.y() : stepSizeParameters.reduceTranslationYFromRotationFast;
  const float tFactorY = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceTranslationFromRotation) / (reduceTransitionY - useReduceTranslationFromRotation))));
  return Vector2f(std::min(tFactorX, std::abs(walkSpeedRatio.x())), std::min(tFactorY, std::abs(walkSpeedRatio.y())));
}

Rangea WalkingEngine::getMaxRotationToStepFactor(const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& stepRatio)
{
  // rotation based on y-translation
  const float useReduceTranslationFromRotation = !isFastWalk ? stepSizeParameters.reduceTranslationFromRotation.y() : stepSizeParameters.reduceTranslationYFromRotationFast;
  const float reduceTransitionYNeg = !isFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : (!isLeftPhase ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
  const float reduceTransitionYPos = !isFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : (isLeftPhase ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
  const Angle minRotY = -std::sqrt(std::max(0.f, -(stepRatio.y() - 1.f))) * std::max(reduceTransitionYNeg - useReduceTranslationFromRotation, 0.f) - useReduceTranslationFromRotation;
  const Angle maxRotY = std::sqrt(std::max(0.f, -(stepRatio.y() - 1.f))) * std::max(reduceTransitionYPos - useReduceTranslationFromRotation, 0.f) + useReduceTranslationFromRotation;

  // rotation based on x-translation
  const Angle useReduceFactor = std::max(customReduceOffset, stepSizeParameters.reduceTranslationFromRotation.x());
  const Angle rotX = std::sqrt(std::max(0.f, -(stepRatio.x() - 1.f))) * (stepSizeParameters.noTranslationFromRotation.x() - useReduceFactor) + useReduceFactor;

  return Rangea(std::max(minRotY, -rotX), std::min(maxRotY, rotX));
}

void WalkingEngine::filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal)
{
  // adjust y forward
  Geometry::Line lineForwardAdjusted(polygonIn[1], (polygonIn[1] - polygonIn[2]).normalized());
  Vector2f leftY;
  if(Geometry::getIntersectionOfLineAndConvexPolygon(polygonOriginal, lineForwardAdjusted, leftY, false))
  {
    polygonIn[1].y() = std::min(leftY.y(), polygonIn[0].y());
    polygonIn[2].y() = std::max(-leftY.y(), polygonIn[3].y());
  }

  // adjust y backward
  Geometry::Line lineBackAdjusted(polygonIn[5], (polygonIn[6] - polygonIn[5]).normalized());
  if(Geometry::getIntersectionOfLineAndConvexPolygon(polygonOriginal, lineBackAdjusted, leftY, false))
  {
    polygonIn[5].y() = std::max(-leftY.y(), polygonIn[4].y());
    polygonIn[6].y() = std::min(leftY.y(), polygonIn[7].y());
  }

  polygonOut.clear();

  for(size_t i = 0; i < polygonIn.size(); ++i)
  {
    const Vector2f& p1 = polygonIn[i];
    const Vector2f& p2 = polygonIn[(i + 1) % polygonIn.size()];
    if(p1 != p2)
      polygonOut.emplace_back(p1);
  }
}

void WalkingEngine::generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool useMaxPossibleStepSize)
{
  ASSERT(!translationPolygon.empty());
  ASSERT(!translationPolygonBig.empty());
  const std::vector<Vector2f>& original = !useMaxPossibleStepSize ? translationPolygon : translationPolygonBig;
  ASSERT(original.size() == 8);
  std::vector<Vector2f> translationPolygonTemp = original;
  for(Vector2f& edge : translationPolygonTemp)
  {
    // x
    if(edge.x() >= 0.f)
      edge.x() = std::min(edge.x(), frontLeft.x());
    else
      edge.x() = std::max(edge.x(), backRight.x());

    // y
    if(edge.y() >= 0.f)
      edge.y() = std::min(edge.y(), frontLeft.y());
    else
      edge.y() = std::max(edge.y(), backRight.y());
  }

  if(frontLeft.x() < 0.f)
  {
    translationPolygonTemp[0].x() = translationPolygonTemp[1].x() = std::max(original[7].x(), translationPolygonTemp[0].x());
    translationPolygonTemp[2].x() = translationPolygonTemp[3].x() = std::max(original[4].x(), translationPolygonTemp[3].x());
  }
  if(backRight.x() > 0.f)
  {
    translationPolygonTemp[7].x() = translationPolygonTemp[6].x() = std::min(original[0].x(), translationPolygonTemp[7].x());
    translationPolygonTemp[4].x() = translationPolygonTemp[5].x() = std::min(original[3].x(), translationPolygonTemp[4].x());
  }

  ASSERT(translationPolygonTemp[3].x() == translationPolygonTemp[0].x());
  ASSERT(translationPolygonTemp[4].x() == translationPolygonTemp[7].x());

  // back right
  translationPolygonTemp[4].x() = translationPolygonTemp[7].x();

  filterTranslationPolygon(polygon, translationPolygonTemp, original);
}

void WalkingEngine::update(StandGenerator& standGenerator)
{
  standGenerator.createPhase = [this](const MotionRequest&, const MotionPhase& lastPhase)
  {
    return std::make_unique<WalkPhase>(*this, Pose2f(), lastPhase);
  };
}

void WalkingEngine::update(WalkingEngineOutput& walkingEngineOutput)
{
  Pose2f useSpeed;
  const float useRobotStateFactor = useJointPlayScaling ? theJointPlay.qualityOfRobotHardware : 1.f;
  useSpeed.rotation = configuredParameters.maxSpeed.rotation * useRobotStateFactor + configuredParameters.minSpeed.rotation * (1.f - useRobotStateFactor);
  useSpeed.translation = configuredParameters.maxSpeed.translation * useRobotStateFactor + configuredParameters.minSpeed.translation * (1.f - useRobotStateFactor);
  const float useSpeedBackwards = configuredParameters.maxSpeedBackwards * useRobotStateFactor + configuredParameters.minSpeedBackwards * (1.f - useRobotStateFactor);

  walkingEngineOutput.maxSpeed = useSpeed;
  walkingEngineOutput.maxSpeedBackwards = useSpeedBackwards;
  walkingEngineOutput.walkStepDuration = kinematicParameters.baseWalkPeriod / 1000.f;

  walkingEngineOutput.maxStepSize = Pose2f(useSpeed.rotation * walkingEngineOutput.walkStepDuration,
                                           useSpeed.translation.x() * walkingEngineOutput.walkStepDuration,
                                           useSpeed.translation.y() * walkingEngineOutput.walkStepDuration * 2.f);
  walkingEngineOutput.maxBackwardStepSize = useSpeedBackwards * walkingEngineOutput.walkStepDuration;

  walkingEngineOutput.maxPossibleStepSize = Pose2f(configuredParameters.walkSpeedParams.maxSpeed.rotation * walkingEngineOutput.walkStepDuration,
                                                   configuredParameters.walkSpeedParams.maxSpeed.translation.x() * walkingEngineOutput.walkStepDuration,
                                                   configuredParameters.walkSpeedParams.maxSpeed.translation.y() * walkingEngineOutput.walkStepDuration * 2.f);
  walkingEngineOutput.maxPossibleBackwardStepSize = configuredParameters.walkSpeedParams.maxSpeedBackwards * walkingEngineOutput.walkStepDuration;

  walkingEngineOutput.energyEfficientWalkSpeed = kinematicParameters.energyEfficientWalkSpeed.maxSpeed;
  walkingEngineOutput.noEfficientWalkSpeed = kinematicParameters.noEfficientWalkSpeed.maxSpeed;
}

float WalkingEngine::getSideSpeed(const float sideTarget) const
{
  if(kinematicParameters.sidewaysWalkHeightPeriodIncreaseFactor > 0.f)
    return (std::sqrt(2000.f * kinematicParameters.sidewaysWalkHeightPeriodIncreaseFactor * std::abs(sideTarget) + kinematicParameters.baseWalkPeriod * kinematicParameters.baseWalkPeriod) - kinematicParameters.baseWalkPeriod) / (2.f * kinematicParameters.sidewaysWalkHeightPeriodIncreaseFactor);
  return 500.f * std::abs(sideTarget) / kinematicParameters.baseWalkPeriod;
}

float WalkingEngine::stepDurationSpeedTarget(const float sideSpeed) const
{
  return (kinematicParameters.baseWalkPeriod + std::abs(sideSpeed) * kinematicParameters.sidewaysWalkHeightPeriodIncreaseFactor) / 1000.f;
}

float WalkingEngine::updateWalkHipHeight(const Pose2f& walkStep, const float stepDuration) const
{
  if(!theMotionRequest.energySavingWalk || !configuredParameters.energySavingWalk || theMotionRequest.shouldInterceptBall)
    return kinematicParameters.walkHipHeight.min;

  if(stepDuration == 0.f)
    return kinematicParameters.walkHipHeight.max;

  return std::min(
  {
    mapToRange(std::abs(walkStep.rotation / stepDuration), static_cast<float>(kinematicParameters.energyEfficientWalkSpeed.maxSpeed.rotation), static_cast<float>(kinematicParameters.noEfficientWalkSpeed.maxSpeed.rotation), kinematicParameters.walkHipHeight.max, kinematicParameters.walkHipHeight.min),
    mapToRange(std::abs(walkStep.translation.x() / stepDuration), kinematicParameters.energyEfficientWalkSpeed.maxSpeed.translation.x(), kinematicParameters.noEfficientWalkSpeed.maxSpeed.translation.x(), kinematicParameters.walkHipHeight.max, kinematicParameters.walkHipHeight.min),
    mapToRange(std::abs(walkStep.translation.y() / (stepDuration * 2.f)), kinematicParameters.energyEfficientWalkSpeed.maxSpeed.translation.y(), kinematicParameters.noEfficientWalkSpeed.maxSpeed.translation.y(), kinematicParameters.walkHipHeight.max, kinematicParameters.walkHipHeight.min)
  });
}

WalkPhase::WalkPhase(WalkingEngine& engine, Pose2f useStepTarget, const MotionPhase& lastPhase, const bool isDelay,
                     const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback,
                     const WalkKickStep& walkKickStep) :
  WalkPhaseBase(engine, walkKickStep, isDelay),
  createNextPhaseCallback(createNextPhaseCallback)
{
  currentWalkHipHeight = engine.kinematicParameters.walkHipHeight.min;
  std::vector<Vector2f> dummyPolygon;
  engine.theWalkGenerator.getTranslationPolygon(isLeftPhase, 0, lastPhase, Pose2f(1.f, 1.f, 1.f), previousInterceptTranslationPolygon, dummyPolygon, false, true);

  if(std::isnan(useStepTarget.translation.x()) || std::isnan(useStepTarget.translation.y()) || std::isnan(useStepTarget.rotation))
  {
    ASSERT(!std::isnan(useStepTarget.translation.x()));
    ASSERT(!std::isnan(useStepTarget.translation.y()));
    ASSERT(!std::isnan(useStepTarget.rotation));
    // make sure in Release the robot does not destroy itself
    ANNOTATION("WalkingEngine", "Step target was nan!");
    OUTPUT_ERROR("WalkingEngine: Step target was nan!");
    useStepTarget = Pose2f();
  }
  bool afterKickOrGetUp = (lastPhase.type == MotionPhase::kick || engine.theKeyframeMotionGenerator.wasLastGetUp(lastPhase) || lastPhase.type == MotionPhase::photoMode);
  if(lastPhase.type == MotionPhase::freeze)
  {
    const Rangef acceptedHeightRange(-currentWalkHipHeight - 5.f, -currentWalkHipHeight + engine.kinematicParameters.baseFootLift + 5.f);
    afterKickOrGetUp = acceptedHeightRange.isInside(engine.theRobotModel.soleLeft.translation.z()) && acceptedHeightRange.isInside(engine.theRobotModel.soleRight.translation.z());
  }
  if(afterKickOrGetUp)
    timeWhenLastKick = engine.theFrameInfo.time;
  const bool standRequested = (useStepTarget.translation.x() == 0.f && useStepTarget.translation.y() == 0.f && useStepTarget.rotation == 0_deg) || (!engine.theGroundContactState.contact && !afterKickOrGetUp);

  isInWalkKickTransitionAllowed = lastPhase.type == MotionPhase::walk && !engine.theWalkGenerator.wasLastPhaseInWalkKick(lastPhase);
  timeWhenStandBegan = engine.theFrameInfo.time;
  leftArmInterpolationStart = engine.theFrameInfo.time;
  rightArmInterpolationStart = engine.theFrameInfo.time;
  leftArm = engine.theJointRequest;
  rightArm = engine.theJointRequest;
  lastJointRequest = engine.theJointRequest;
  Pose2f lastStepTarget;
  standInterpolationDuration = 1000.f; // default value
  for(std::size_t i = 0; i < armCompensationTilt.capacity(); i++)
    armCompensationTilt.push_front(0);

  if(timeWhenLastKick > engine.theFrameInfo.time)
    timeWhenLastKick = engine.theFrameInfo.time;

  leftArmInterpolationTime = rightArmInterpolationTime = engine.armParameters.armInterpolationTime;
  speedControlParams = engine.speedControlParams;

  // Previous phase was a walk phase
  if(lastPhase.type == MotionPhase::walk)
  {
    if(afterKickOrGetUp)
    {
      this->createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback();
      this->walkKickStep = WalkKickStep();
    }
    constructorWalkCase(lastPhase, useStepTarget, lastStepTarget, standRequested);
  }

  // Previous phase was a stand phase
  else if(lastPhase.type == MotionPhase::stand)
    constructorStandCase(standRequested, lastPhase, useStepTarget);

  // Previous phase was a kick or get up phase
  else if(afterKickOrGetUp)
    constructorAfterKickOrGetUpCase(lastPhase);

  // Previous phase was something else (e.g. playDead)
  else
    constructorOtherCase();

  // Previous was walking (or stopping). Calculate step size. Rotational requests must be compensated
  if(walkState != standing)
    constructorHelperInitWalkPhase(useStepTarget, lastStepTarget, afterKickOrGetUp, standRequested, lastPhase);
  else
    type = MotionPhase::stand;

  if(!(lastPhase.type == MotionPhase::stand && type == MotionPhase::stand))
    engine.theEnergySaving.shutDown();

  /*if (lastPhase.type == MotionPhase::stand && type == MotionPhase::stand)
    OUTPUT_ERROR("Stand phase followed by a stand phase!");*/

  constructorWeightShiftStatus();

  // Prevent false time stamps when replaying logs
  if(engine.theFrameInfo.time < leftArmInterpolationStart)
    leftArmInterpolationStart = engine.theFrameInfo.time;
  if(engine.theFrameInfo.time < rightArmInterpolationStart)
    rightArmInterpolationStart = engine.theFrameInfo.time;

  // Set walk values
  engine.theWalkStepData.updateWalkValues(step, stepDuration, isLeftPhase);

  // Calc relative ball
  calculateBallPosition(isLeftPhase, useStepTarget);
  constructorJointSpeedController();
  initMaxRotationSpeedPerMotionStep(useStepTarget);

  // After kicks or after a freeze allow some clipping in the inverse kinematic
  increaseInverseKinematicClipThreshold = lastPhase.type == MotionPhase::kick || lastPhase.type == MotionPhase::freeze || afterWalkKickPhase;
  originalStep = step;
  measuredSideL0 = engine.theRobotModel.soleLeft.translation.y() - engine.theRobotDimensions.yHipOffset - engine.kinematicParameters.yHipOffset;
  measuredSideR0 = engine.theRobotModel.soleRight.translation.y() + engine.theRobotDimensions.yHipOffset + engine.kinematicParameters.yHipOffset;

  targetWalkHipHeight = engine.updateWalkHipHeight(step, stepDuration);

  supportForwardCompensation = LowPassFilterPR(engine.supportCompensationParameters.lowPassFilterSlow, engine.supportCompensationParameters.lowPassFilterFast);
}

void WalkPhase::update()
{
  // Stop moving if swing foot has ground contact
  if(tBase < engine.minPhaseForStopWithWrongGroundContact * stepDuration || (
       isLeftPhase && !engine.theSolePressureState.legInfo[Legs::left].hasPressure) ||
     (!isLeftPhase && !engine.theSolePressureState.legInfo[Legs::right].hasPressure))
  {
    if((engine.balanceParameters.allowSlowDownWhenWalkingBackwards && step.translation.x() < 0.f) || step.translation.x() >= 0.f)
      tWalk += engine.motionCycleTime * (1.f - engine.balanceParameters.slowdownFactor * Rangef::ZeroOneRange().limit((engine.theInertialData.angle.y() + engine.balanceParameters.slowdownTorsoOffset + armCompensationTilt[0]) / -engine.balanceParameters.minTorsoRotation));
    else if(!engine.balanceParameters.allowSlowDownWhenWalkingBackwards && step.translation.x() < 0.f)
      tWalk += engine.motionCycleTime * (1.f - engine.balanceParameters.slowdownFactor * Rangef::ZeroOneRange().limit((engine.theInertialData.angle.y() - engine.balanceParameters.slowdownTorsoOffset + armCompensationTilt[0]) / engine.balanceParameters.minTorsoRotation));
    else
      tWalk += engine.motionCycleTime;
    tWalkSide += freezeSideMovement ? 0.f : engine.motionCycleTime;
  }
  tBase += engine.motionCycleTime;

  // Update walk step
  updateDynamicStep(); // TODO eval if there was a reason this was previously after the jointSpeedController update

  // Update walk height
  Rangef heightInterpolationSpeed = engine.kinematicParameters.hipHeightChangeSpeed;
  heightInterpolationSpeed.min *= engine.motionCycleTime;
  heightInterpolationSpeed.max *= engine.motionCycleTime;
  currentWalkHipHeight += heightInterpolationSpeed.limit((targetWalkHipHeight - currentWalkHipHeight));
  ASSERT(engine.kinematicParameters.walkHipHeight.isInside(currentWalkHipHeight));

  // Save last values
  prevTurn = turnRL;
  lastPrevSideL = prevSideL;
  lastPrevSideR = prevSideR;
  prevSideL = sideL;
  prevSideR = sideR;

  // Update ball prediction time
  if(walkKickStep.currentKickVariant.has_value())
    walkKickStep.currentKickVariant->ballEstimationTime = std::max(0.f, walkKickStep.currentKickVariant->ballEstimationTime - engine.motionCycleTime);

  // Update support switch info
  const bool isSwitchAllowed = tBase > engine.configuredParameters.supportSwitchPhaseRange.min * stepDuration;
  // enough time passed, prediction is allowed, prediction predicts an actual switch
  supportSwitchInfo.isPredictedSwitch = engine.useFootSupportSwitchPrediction && isSwitchAllowed && engine.theFootSupport.trustedSupport && engine.theFootSupport.predictedSwitched && // prediction is active
                                        (std::abs(step.translation.angle()) < engine.maxWalkDirectionForFootPrediction && // not if walking too much backwards
                                         walkKickStep.currentKick == WalkKicks::none && isLeftPhase != engine.theFootSupport.support > 0.f); // only for non kicks and if current used support foot is still the same

  const bool wrongSupportFoot = (engine.theFootSupport.support < 0.f != isLeftPhase);
  wrongSupportFootCounter += wrongSupportFoot && tBase > engine.minTimeForEarlySwitch && engine.theFootSupport.trustedSupport ? 1 : 0;
  supportSwitchInfo.isAbortSwitch = wrongSupportFootCounter >= 3 && walkKickStep.currentKick == WalkKicks::none;
  supportSwitchInfo.isStuckSwitch = canForceFootSupportSwitch();
  supportSwitchInfo.isFeetStepAbort = tBase > stepDuration && engine.theFrameInfo.getTimeSince(engine.theIMUValueState.gyroValues.stableSinceTimestamp) >= engine.configuredParameters.emergencyNotMovingTime;
  supportSwitchInfo.isNormalSwitch = isSwitchAllowed && engine.theFootSupport.switched;
  supportSwitchInfo.isOvertimeSwitch = tBase > engine.configuredParameters.supportSwitchPhaseRange.max * stepDuration;
  supportSwitchInfo.isFootSupportSwitch = (supportSwitchInfo.isAbortSwitch || supportSwitchInfo.isFeetStepAbort || supportSwitchInfo.isNormalSwitch || supportSwitchInfo.isOvertimeSwitch) && !supportSwitchInfo.isPredictedSwitch;
}

bool WalkPhase::isDone(const MotionRequest& motionRequest) const
{
  const bool normalIsDone = walkState == standing ? ((motionRequest.motion != MotionRequest::stand || motionRequest.shouldInterceptBall) && ((!motionRequest.isWalking() && !motionRequest.shouldInterceptBall) || (engine.theGroundContactState.contact && standFactor == 0.f))) // switch from stand to walk
                            : (supportSwitchInfo.isPredictedSwitch || supportSwitchInfo.isAbortSwitch || supportSwitchInfo.isNormalSwitch ||
                               supportSwitchInfo.isOvertimeSwitch || supportSwitchInfo.isFeetStepAbort || supportSwitchInfo.isStuckSwitch);
  if(normalIsDone)
    return true;
  const bool diveStart = motionRequest.motion == MotionRequest::dive && (motionRequest.diveRequest == MotionRequest::Dive::jumpLeft || motionRequest.diveRequest == MotionRequest::Dive::jumpRight) &&
                         isStandingPossible(forwardL, forwardR, lastPrevSideL, lastPrevSideR, turnRL, walkStepAdjustment.lastLeftAdjustmentX,
                                            walkStepAdjustment.lastRightAdjustmentX, walkStepAdjustment.highestAdjustmentX, sideHipShift, true);
  if(diveStart)
    ANNOTATION("WalkingEngine", "Early step stop for keeper motion");
  return diveStart;
}

void WalkPhase::updateDynamicStep()
{
  if(walkState == standing || walkKickStep.currentKick != WalkKicks::none || (!engine.configuredParameters.dynamicStepUpdate && !engine.configuredParameters.dynamicKickPoseUpdate))
    return;

  // Robot shall intercept the ball and is still at the beginning of the walk step. Start intercept or update the already executed intercept step
  if((engine.theMotionRequest.shouldInterceptBall || wasInterceptingLastFrame) && !engine.theMotionRequest.shouldWalkOutOfBallLine && tBase > 1.5f * engine.motionCycleTime)
  {
    ASSERT(previousInterceptTranslationPolygon.size() >= 4);
    Pose2f nextStep = originalStep;
    if(engine.theMotionRequest.shouldInterceptBall)
    {
      if(engine.configuredParameters.dynamicStepUpdate)
        nextStep = engine.theInterceptBallGenerator.intercept(engine.theMotionRequest, previousInterceptTranslationPolygon, isLeftPhase, step.translation);
      else
      {
        if(!wasInterceptingLastFrame)  // update polygon, because we want to ensure the x-translation size is higher than normal
        {
          // first get highest x translations
          // second get highest values from previous translation polygon
          const float useRobotStateFactor = engine.useJointPlayScaling ? engine.theJointPlay.qualityOfRobotHardware : 1.f;
          float yMin, yMax = 0.f;
          const float useMinXBackwardTranslation = engine.stepSizeParameters.minXBackwardTranslationFastRange.min * (1.f - useRobotStateFactor) + engine.stepSizeParameters.minXBackwardTranslationFastRange.max * useRobotStateFactor;
          float xMax = engine.stepSizeParameters.minXForwardTranslationFast * engine.theMotionRequest.walkSpeed.translation.x();
          float xMin = useMinXBackwardTranslation * engine.theMotionRequest.walkSpeed.translation.x();
          for(const Vector2f& v : previousInterceptTranslationPolygon)
          {
            xMin = std::min(v.x(), xMin);
            yMin = std::min(v.y(), yMin);
            xMax = std::max(v.x(), xMax);
            yMax = std::max(v.y(), yMax);
          }
          engine.generateTranslationPolygon(previousInterceptTranslationPolygon, Vector2f(xMin, yMin), Vector2f(xMax, yMax), false);
        }
        nextStep = engine.theInterceptBallGenerator.kickIntercept(engine.theMotionRequest, previousInterceptTranslationPolygon, isLeftPhase, step.translation);
      }
    }

    // Make sure the step is valid
    if(std::isnan(nextStep.translation.x()) || std::isnan(nextStep.translation.y()) || std::isnan(nextStep.rotation))
    {
      FAIL("x: " << nextStep.translation.x() << " y: " << nextStep.translation.y() << " rotation: " << nextStep.rotation);
      // make sure in Release the robot does not destroy itself
      ANNOTATION("WalkingEngine", "Step target was nan!");
      OUTPUT_ERROR("WalkingEngine: Step target was nan! " << "x: " << nextStep.translation.x() << " y: " << nextStep.translation.y() << " rotation: " << nextStep.rotation);
    }
    else
    {
      // Interpolate between old and new one
      const Pose2f useRefStep = step;
      const float interpolationFactor = engine.theMotionRequest.shouldInterceptBall && wasInterceptingLastFrame ? 1.f - engine.motionCycleTime / stepDuration : Rangef::ZeroOneRange().limit(tBase / stepDuration);
      const float framesLeft = std::max(stepDuration - tBase, engine.motionCycleTime) / engine.motionCycleTime;
      const float maxSideSpeed = engine.configuredParameters.walkSpeedParams.maxSpeed.translation.y() * engine.motionCycleTime;
      const float maxSideSpeedAcc = maxSideSpeed / 4.f; // do not calculate the correct value into the future, because the real foot shall change the velocity
      const Rangef maxSideSpeedRange(-maxSideSpeed, maxSideSpeed);

      auto calcSideAdjustment = [&]()
      {
        if(engine.theMotionRequest.shouldInterceptBall)
          return framesLeft * sideAcc;

        // This code below is only for the case the intercepting is stopping again
        const float supportCurrent = isLeftPhase ? sideR : sideL;
        const float supportNewTarget = ((-nextStep.translation.y() + sideHipShift) * engine.kinematicParameters.sidewaysHipShiftFactor);
        const float newDirectionSign = (supportNewTarget - supportCurrent > 0.f ? -1.f : 1.f);
        if(sideAcc * newDirectionSign < 0.f)
          sideAcc = maxSideSpeedAcc * newDirectionSign;
        float sideChange = sideAcc;
        float sideAdjustment = 0.f;
        for(int i = 0; i < framesLeft; i++)
        {
          sideAdjustment += sideChange;
          sideChange = maxSideSpeedRange.limit(sideChange + maxSideSpeedAcc * newDirectionSign);
        }
        return sideAdjustment;
      };

      const float sideAdjustment = calcSideAdjustment();
      const Rangef sideAdjustmentRange(std::min(sideAdjustment + step.translation.y(), step.translation.y()), std::max(sideAdjustment + step.translation.y(), step.translation.y()));
      const Pose2f originalNextStep = nextStep;
      nextStep.translation.x() = (1.f - interpolationFactor) * nextStep.translation.x() + interpolationFactor * useRefStep.translation.x();
      nextStep.translation.y() = sideAdjustmentRange.limit(nextStep.translation.y());
      nextStep.rotation = (1.f - interpolationFactor) * nextStep.rotation + interpolationFactor * useRefStep.rotation;

      {
        // we can use the current walk variables, because we know at least one walk frame was already executed
        // this code is only useful when we continue to intercept
        const float supportCurrent = isLeftPhase ? sideR : sideL;
        const float supportNewTarget = ((-originalNextStep.translation.y() + sideHipShift) * engine.kinematicParameters.sidewaysHipShiftFactor);
        const float newDirectionSign = (supportNewTarget - supportCurrent > 0.f ? -1.f : 1.f);
        if(sideAcc * newDirectionSign < 0.f)
          sideAcc = maxSideSpeedAcc * newDirectionSign;
        else
          sideAcc = maxSideSpeedRange.limit(sideAcc + maxSideSpeedAcc * newDirectionSign);
      }

      const Pose2f previousStep = step;
      step = nextStep;

      // Clip step
      if(!Geometry::isPointInsideConvexPolygon(previousInterceptTranslationPolygon.data(), static_cast<int>(previousInterceptTranslationPolygon.size()), step.translation) && previousStep.translation != Vector2f(step.translation.x(), step.translation.y()))
      {
        Vector2f intersectionPointBase, intersectionPoint;
        if(Geometry::isPointInsideConvexPolygon(previousInterceptTranslationPolygon.data(), static_cast<int>(previousInterceptTranslationPolygon.size()), previousStep.translation))
          intersectionPointBase = previousStep.translation * 0.99f;
        else
        {
          VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(previousInterceptTranslationPolygon, Geometry::Line(Vector2f(0.f, 0.f), previousStep.translation), intersectionPointBase, false));
          intersectionPointBase *= 0.99f;
        }
        VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(previousInterceptTranslationPolygon, Geometry::Line(intersectionPointBase, step.translation - intersectionPointBase), intersectionPoint, false));
        step.translation = intersectionPoint * 0.99f; // 99% to ensure the point is inside the polygon
      }

      const Pose2f convertedNextStep = convertStep(nextStep, 0_deg);
      turnStep = convertedNextStep.rotation;
      forwardStep = convertedNextStep.translation.x();
      sideStep = convertedNextStep.translation.y();

      engine.theWalkStepData.updateWalkValues(step, stepDuration, isLeftPhase);
      wasInterceptingLastFrame = engine.theMotionRequest.shouldInterceptBall;

      // Need to update it. Otherwise the robot could have played moving the feet together and now wanting to intercept the ball.
      // This results in a large side step, but the old hip shift makes the feet pose unreachable
      if(sideHipShift != 0.f)
        updateSideHipShift(false);

      targetWalkHipHeight = engine.updateWalkHipHeight(step, stepDuration);
    }
  }
}

void WalkPhase::transitionWalkToKick()
{
  // Do not overwrite current kick and also a transition is not allowed right now
  if(engine.theMotionRequest.motion != MotionRequest::walkToBallAndKick || // dribbling not supported right now
     engine.theMotionRequest.alignPrecisely != KickPrecision::justHitTheBall || // only for kicks that need to be executed fast
     !engine.configuredParameters.transitionWalkToKick || walkKickStep.currentKick != WalkKicks::none ||
     !isInWalkKickTransitionAllowed || tBase > Global::getSettings().motionCycleTime * 3.5f)
    return;

  // Only InWalkKicks are allowed
  switch(engine.theMotionRequest.kickType)
  {
    case KickInfo::walkForwardsLeft:
    case KickInfo::walkForwardsLeftLong:
    case KickInfo::walkForwardsRight:
    case KickInfo::walkForwardsRightLong:
    case KickInfo::walkSidewardsRightFootToRight:
    case KickInfo::walkTurnRightFootToLeft:
    case KickInfo::walkSidewardsLeftFootToLeft:
    case KickInfo::walkTurnLeftFootToRight:
    case KickInfo::walkForwardStealBallLeft:
    case KickInfo::walkForwardStealBallRight:
    case KickInfo::walkForwardsRightAlternative:
    case KickInfo::walkForwardsLeftAlternative:
      break;
    default:
      return;
  }

  // Check whether the kick could start
  WalkKickVariant walkKickVariant(engine.theMotionRequest.kickType, engine.theKickInfo[engine.theMotionRequest.kickType].walkKickType, engine.theKickInfo[engine.theMotionRequest.kickType].kickLeg, KickPrecision::justHitTheBall, engine.theMotionRequest.kickLength, engine.theMotionRequest.targetDirection, engine.theMotionRequest.shiftTurnKickPose);
  walkKickVariant.forceSwingFoot = isLeftPhase;
  if(engine.theWalkKickGenerator.canStart(walkKickVariant, *lastPhase, engine.theMotionRequest.directionPrecision, PreStepType::notAllowed, engine.theMotionRequest.turnKickAllowed))
  {
    // Create walk kick phase
    std::unique_ptr<MotionPhase> otherWalkPhase = engine.theWalkKickGenerator.createPhase(walkKickVariant, *lastPhase, true);
    if(otherWalkPhase)
    {
      // Copy walkKickStep
      const auto& lastWalkPhaseDummy = static_cast<const WalkPhase&>(*otherWalkPhase);
      walkKickStep = lastWalkPhaseDummy.walkKickStep;
      createNextPhaseCallback = lastWalkPhaseDummy.createNextPhaseCallback;

      forwardStep = this->walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.translation.x();
      sideStep = this->walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.translation.y();
      turnStep = this->walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.rotation;
      maxFootHeight = engine.kinematicParameters.baseFootLift * walkKickStep.increaseSwingHeightFactor;
      kickType = walkKickStep.currentKickVariant->kickType;
    }
  }
}

void WalkPhase::calculateCurrentBaseStepVariables()
{
  const float stepDurationOffset = SystemCall::getMode() == SystemCall::simulatedRobot ? engine.motionCycleTime * 3.f : 0.f;

  // Walking
  if(walkKickStep.currentKick == WalkKicks::none)
  {
    if(isLeftPhase)
      calcFootOffsets(1.f, tWalk, tWalkSide, tBase, stepDuration - stepDurationOffset, stepHeightDuration, forwardL0, forwardR0, forwardL, forwardR, sideL0, sideR0, sideL, sideSwingStretch, sideR, footHL0, footHR0, footHL, footHR, turnRL, forwardStep, sideStep, turnStep);
    else
      calcFootOffsets(-1.f, tWalk, tWalkSide, tBase, stepDuration - stepDurationOffset, stepHeightDuration, forwardR0, forwardL0, forwardR, forwardL, sideR0, sideL0, sideR, sideSwingStretch, sideL, footHR0, footHL0, footHR, footHL, turnRL, forwardStep, sideStep, turnStep);
  }
  // InWalkKicks
  else
  {
    if(isLeftPhase)
      calcWalkKickFootOffsets(1.f, tBase, forwardL0, forwardR0, forwardL, forwardR, sideL0, sideR0, sideL, sideSwingStretch, sideR, footHL0, footHR0, footHL, footHR, turnRL, walkKickStep, turnRL0);
    else
      calcWalkKickFootOffsets(-1.f, tBase, forwardR0, forwardL0, forwardR, forwardL, sideR0, sideL0, sideR, sideSwingStretch, sideL, footHR0, footHL0, footHR, footHL, turnRL, walkKickStep, turnRL0);
  }
}

void WalkPhase::calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  // Predict CoM position
  engine.theRobotStableState.predictRotation(isLeftPhase, !afterWalkKickPhase, false);

  ASSERT(engine.kinematicParameters.walkHipHeight.isInside(currentWalkHipHeight));

  transitionWalkToKick();

  JointRequest other = engine.theJointRequest;
  other.angles = engine.theJointAngles.angles;
  // Update the joints that are predicted.
  if(engine.theJointAnglePred.isValid)
  {
    FOREACH_ENUM(Joints::Joint, joint)
    {
      if(engine.theJointAnglePred.angles[joint] != SensorData::ignore)
        other.angles[joint] = engine.theJointAnglePred.angles[joint];
    }
  }
  const RobotModel requestedModelPred(other, engine.theRobotDimensions, engine.theMassCalibration);
  const Angle supportSoleRotY = (isLeftPhase ? requestedModelPred.soleRight : requestedModelPred.soleLeft).rotation.getYAngle();

  // "Correction" factor for simulation. This helps the sim NAO to behave more like the real one
  const Angle supportSoleRotErrorMeasured = supportSoleRotY - armCompensationTilt[armCompensationTilt.size() - 2];

  if(walkState != standing)
  {
    // TODO move jointSpeedController stuff into own function and call it in WalkPhase::update()
    // Save values, as those can change after the update step
    const float oldFootHL0 = footHL0;
    const float oldFootHR0 = footHR0;
    const float oldForwardL0 = forwardL0;
    const float oldForwardR0 = forwardR0;
    const float oldSideL0 = sideL0;
    const float oldSwingStretch = sideSwingStretch;
    const float oldSideR0 = sideR0;
    const Angle oldTurnRL0 = turnRL0;
    const WalkKickStep oldWalkKick = walkKickStep;

    calculateCurrentBaseStepVariables();

    // Calculate turnRL reduction
    const auto& ratioRef = engine.theRobotStableState.comInFeet[isLeftPhase ? Legs::right : Legs::left];
    const auto& ratioRefTorso = engine.theRobotStableState.comInTorso[isLeftPhase ? Legs::right : Legs::left];
    jointSpeedController->updateZRotation(speedControlParams, supportSoleRotErrorMeasured, ratioRef.forward, ratioRefTorso.forward, engine.theJointPlay.qualityOfRobotHardware, engine.filteredGyroY);
    const Angle preTurnRL = turnRL;
    jointSpeedController->clipZRotation(turnRL);
    const Angle reducedTurn = (turnRL - preTurnRL) * (isLeftPhase ? 1.f : -1.f) * 2.f;

    // Copy back here, as it is needed for the step conversion afterwards
    walkKickStep = oldWalkKick;
    // Correct translation step size, as target turn has changed
    if(walkKickStep.currentKick == WalkKicks::none)
    {
      const Pose2f convertedStep = convertStep(step, reducedTurn);
      forwardStep = convertedStep.translation.x();
      sideStep = convertedStep.translation.y();
    }
    else // A bit more to do for InWalkKicks
    {
      for(size_t i = 0; i < walkKickStep.keyframe.size(); i++)
      {
        if(walkKickStep.keyframe[i].reachedWaitPosition)
          continue;

        // Main Step
        const Pose2f& useStepTarget = this->walkKickStep.keyframe[i].stepTarget;
        Pose2f nextStep = convertStep(useStepTarget, reducedTurn);
        this->walkKickStep.keyframe[i].stepTargetConverted.translation = nextStep.translation;
        this->walkKickStep.keyframe[i].stepTargetConverted.rotation = this->walkKickStep.keyframe[i].stepTarget.rotation;

        // Offset swing step
        const Pose2f& useStepTargetSwing = this->walkKickStep.keyframe[i].stepTargetSwing;
        nextStep = convertStep(useStepTargetSwing, reducedTurn);
        this->walkKickStep.keyframe[i].stepTargetSwingConverted.translation = nextStep.translation;
        this->walkKickStep.keyframe[i].stepTargetSwingConverted.rotation = this->walkKickStep.keyframe[i].stepTarget.rotation;
      }
    }

    // Set variables back to original ones
    footHL0 = oldFootHL0;
    footHR0 = oldFootHR0;
    forwardL0 = oldForwardL0;
    forwardR0 = oldForwardR0;
    sideL0 = oldSideL0;
    sideR0 = oldSideR0;
    turnRL0 = oldTurnRL0;
    sideSwingStretch = oldSwingStretch;

    // do update step once again
    calculateCurrentBaseStepVariables();

    jointSpeedController->clipZRotation(turnRL);
  }
  else
  {
    calcJointsHelperInterpolateStanding(motionRequest);
  }

  // Rotate swing foot sole to prevent colliding with the ground
  calcJointsHelperFootSoleRotations(jointRequest, armCompensationTilt[0]);

  Pose3f leftFootForArms;
  Pose3f rightFootForArms;

  engine.calcFeetPoses(forwardL, forwardR, sideL, sideR, footHL, footHR, turnRL, currentWalkHipHeight, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms); // current request

  // Arm swing
  // As the only change can be the arms, and their translation change is independent of the legs, we do not need to know the leg positions yet
  for(std::size_t j = Joints::firstLegJoint; j < Joints::numOfJoints; j++)
    jointRequest.angles[j] = 0_deg;
  JointRequest walkArms = jointRequest;
  setArms(leftFootForArms, rightFootForArms, jointRequest, walkArms);

  // Compensate arm position

  compensateArms(jointRequest, walkArms);

  Pose3f leftFoot = leftFootForArms;
  Pose3f rightFoot = rightFootForArms;

  // PlayDead to Stand
  if(standFactor < 0.f)
  {
    // TODO eval whether the arm compensation should be applied here too
    VERIFY(InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions));
    // Interpolate to stand.
    const float f = 0.5f * (std::sin((standFactor + 0.5f) * pi) - 1.f);
    for(int i = Joints::firstArmJoint; i < Joints::numOfJoints; ++i)
      if(jointRequest.angles[i] != JointAngles::off)
        jointRequest.angles[i] += f * (jointRequest.angles[i] - startJointAngles.angles[i]);
    if(jointRequest.angles[Joints::waistYaw] != JointAngles::off)
      jointRequest.angles[Joints::waistYaw] += f * (jointRequest.angles[Joints::waistYaw] - startJointAngles.angles[Joints::waistYaw]);
  }
  // Walking
  else if(walkState != standing)
  {
    // InWalkKicks and balancing
    balanceFeetPoses(leftFoot, rightFoot);
    calculateSupportSideCompensation();
    leftFoot.translation.x() += isLeftPhase ? supportForwardCompensation.currentValue : 0.f;
    rightFoot.translation.x() += !isLeftPhase ? supportForwardCompensation.currentValue : 0.f;
    if(!InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions, increaseInverseKinematicClipThreshold ? engine.kinematicParameters.legLengthClipThreshold : 10.f))
    {
      OUTPUT_ERROR("Left Foot - Rotations (x y z): " << Angle(leftFoot.rotation.getXAngle()).toDegrees() << " " << Angle(leftFoot.rotation.getYAngle()).toDegrees() << " " << Angle(leftFoot.rotation.getZAngle()).toDegrees() <<
                   " | Left Foot - Translations (x y z): " << leftFoot.translation.x() << " " << leftFoot.translation.y() << " " << leftFoot.translation.z() <<
                   " | Right Foot - Rotations (x y z): " << Angle(rightFoot.rotation.getXAngle()).toDegrees() << " " << Angle(rightFoot.rotation.getYAngle()).toDegrees() << " " << Angle(rightFoot.rotation.getZAngle()).toDegrees() <<
                   " | Right Foot - Translations (x y z): " << rightFoot.translation.x() << " " << rightFoot.translation.y() << " " << rightFoot.translation.z() <<
                   " | Inverse Kinematic Clip Threshold State: " << increaseInverseKinematicClipThreshold);
      ASSERT(false); // Crash when not Release
    }

    if(!Approx::isZero(kneeBalance, 0.1_deg))
    {
      const RobotModel model(jointRequest, engine.theRobotDimensions, engine.theMassCalibration);
      Pose3f leftFoot = model.soleLeft;
      Pose3f rightFoot = model.soleRight;
      Pose3f tibia = model.limbs[isLeftPhase ? Limbs::tibiaLeft : Limbs::tibiaRight];

      Pose3f sole = isLeftPhase ? model.soleLeft : model.soleRight;
      const Pose3f diff = tibia.inverse() * sole;
      sole = tibia.rotateY(kneeBalance) * diff;
      if(isLeftPhase)
        leftFoot = sole;
      else
        rightFoot = sole;
      VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions, increaseInverseKinematicClipThreshold ? engine.kinematicParameters.legLengthClipThreshold : 10.f));
    }
  }
  // Standing
  else
    VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions, increaseInverseKinematicClipThreshold ? engine.kinematicParameters.legLengthClipThreshold : 10.f));

  // Offsets for the InWalkKicks
  applyWalkKickLongKickOffset(jointRequest, tBase);

  // Apply joint speed regulation
  JointAngles speedRegulationChanges = applyJointSpeedControl(jointRequest, supportSoleRotErrorMeasured);
  const JointRequest lastJointRequestCopy = lastJointRequest;
  lastJointRequest = jointRequest;

  // Apply gyro
  const JointAngles appliedGyroChanges = addGyroBalance(jointRequest, lastJointRequestCopy);

  // TODO: test of offset should be applied afterwards
  jointPlayOffsetController->update(engine.theJointPlayTranslation, jointRequest, isLeftPhase, walkState == standing, engine.configuredParameters.jointPlayOffsetParameters,
                                    speedRegulationChanges, appliedGyroChanges, walkState == standing ? 1.f : Rangef::ZeroOneRange().limit(tBase / stepDuration));

  {
    const RobotModel model(jointRequest, engine.theRobotDimensions, engine.theMassCalibration);
    const Pose3f rot(Rotation::aroundY(-armCompensationTilt[0]));
    armPitchShift[Legs::left] = (rot * model.soleLeft).translation.x() - leftFootForArms.translation.x();
    armPitchShift[Legs::right] = (rot * model.soleRight).translation.x() - rightFootForArms.translation.x();
  }

  PLOT("module:WalkingEngine:JointOffset:lHipPitch", jointPlayOffsetController->at(Joints::lHipPitch).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:lKneePitch", jointPlayOffsetController->at(Joints::lKneePitch).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:lAnklePitch", jointPlayOffsetController->at(Joints::lAnklePitch).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:rHipPitch", jointPlayOffsetController->at(Joints::rHipPitch).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:rKneePitch", jointPlayOffsetController->at(Joints::rKneePitch).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:rAnklePitch", jointPlayOffsetController->at(Joints::rAnklePitch).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:rHipRoll", jointPlayOffsetController->at(Joints::rHipRoll).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:rAnkleRoll", jointPlayOffsetController->at(Joints::rAnkleRoll).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:lHipRoll", jointPlayOffsetController->at(Joints::lHipRoll).toDegrees());
  PLOT("module:WalkingEngine:JointOffset:lAnkleRoll", jointPlayOffsetController->at(Joints::lAnkleRoll).toDegrees());

  stiffnessState =
    WalkStiffnessRegulator::getState(walkState, standFactor, engine.theFrameInfo.getTimeSince(timeWhenStandBegan),
                                     engine.theFrameInfo.getTimeSince(timeWhenStandHighBegan), engine.walkStiffnessParameters,
                                     engine.theIMUValueState.notMovingSinceTimestamp != engine.theFrameInfo.time || stiffnessState == StiffnessState::StandStill,
                                     walkKickStep.currentKick != WalkKicks::none);

  WalkStiffnessRegulator::setStiffness(jointRequest, engine.walkStiffnessParameters,
                                       stiffnessState, engine.theGroundContactState.contact, isLeftPhase);

  // Apply stand heat
  if(!(stiffnessState == StiffnessState::Stand || stiffnessState == StiffnessState::StandHigh || stiffnessState == StiffnessState::StandStill || standFactor == 0.f || standFactor == 1.f) &&
     engine.theEnergySaving.state == EnergySaving::EnergyState::working)
    engine.theEnergySaving.reset();
  if((stiffnessState == StiffnessState::Walk || stiffnessState == StiffnessState::Kick) && engine.theEnergySaving.state == EnergySaving::EnergyState::waiting)
    engine.theEnergySaving.shutDown();
  if(stiffnessState == StiffnessState::Stand || stiffnessState == StiffnessState::StandStill || stiffnessState == StiffnessState::StandHigh || engine.theEnergySaving.state != EnergySaving::off)
    engine.theEnergySaving.applyHeatAdjustment(jointRequest, true, true, true, true, standFactor == 1.f, engine.theCalibrationRequest.preciseJointPositions);

  // Odometry
  odometryOffset = getOdometryOffset();

  // Information for the behavior
  motionInfo.isMotionStable = true;
  motionInfo.isWalkPhaseInWalkKick = walkKickStep.currentKick != WalkKicks::none;
  motionInfo.speed = stepDuration > 0.f ? Pose2f(turnStep / stepDuration, forwardStep / stepDuration, sideStep / stepDuration) : Pose2f(); // TODO
}

std::unique_ptr<MotionPhase> WalkPhase::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  // There must be another walk phase if the robot should transition into something other than walking and the feet are not next to each other.
  if(defaultNextPhase.type != MotionPhase::walk && walkState != standing &&
     !isStandingPossible(forwardL, forwardR, lastPrevSideL, lastPrevSideR, turnRL, walkStepAdjustment.lastLeftAdjustmentX,
                         walkStepAdjustment.lastRightAdjustmentX, walkStepAdjustment.highestAdjustmentX, sideHipShift, defaultNextPhase.type == MotionPhase::keyframeMotion))
    return std::make_unique<WalkPhase>(engine, Pose2f(), *this);
  if(std::unique_ptr<MotionPhase> nextPhase; createNextPhaseCallback && (nextPhase = createNextPhaseCallback(*this, this->walkKickStep)))
    return nextPhase;
  return std::unique_ptr<MotionPhase>();
}

void WalkPhase::calcFootOffsets(const float swingSign, const float ratio, const float ratioSide, const float ratioBase,
                                const float duration, const float heightDuration,
                                const float forwardSwing0, const float forwardSupport0, float& forwardSwing,
                                float& forwardSupport, const float sideSwing0, const float sideSupport0,
                                float& sideSwing, float& sideSwingStrecht, float& sideSupport, float& footHeightSwing0,
                                float& footHeightSupport0, float& footHeightSwing,
                                float& footHeightSupport, Angle& turnVal,
                                const float useForwardStep, const float useSideStep, const Angle useTurnStep)
{
  // Side translation
  sideSupport = sideSupport0 + ((-useSideStep + sideHipShift) * engine.kinematicParameters.sidewaysHipShiftFactor - sideSupport0) * Rangef::ZeroOneRange().limit(ratioSide / duration);
  sideSwing = sideSwing0 + ((useSideStep + sideHipShift) * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor) - sideSwing0) * Rangef::ZeroOneRange().limit(ratioSide / duration);

  if(engine.sideCatchWalkParameters.allowedAdjustmentDuringTimeRatio.isInside(ratioBase / duration) && !engine.theSolePressureState.legInfo[isLeftPhase ? Legs::left : Legs::right].hasPressure)
  {
    const Rangef maxSideAdjustment(-engine.sideCatchWalkParameters.maxAdjustment, engine.sideCatchWalkParameters.maxAdjustment);
    const float maxAdjustmentSpeed = engine.configuredParameters.walkSpeedParams.maxSpeed.translation.y() * engine.motionCycleTime * swingSign;
    sideSwingStrecht += mapToRange(engine.theRobotStableState.comInTorso[isLeftPhase ? Legs::left : Legs::right].outerSide, engine.sideCatchWalkParameters.comYThreshold.min, engine.sideCatchWalkParameters.comYThreshold.max, 0.f, maxAdjustmentSpeed);
    sideSwingStrecht = maxSideAdjustment.limit(sideSwingStrecht);
  }

  if(ratioBase > duration + engine.commonSpeedParameters.swingStretchAfterOvertime && !engine.theSolePressureState.legInfo[isLeftPhase ? Legs::left : Legs::right].hasPressure)
  {
    const Rangef maxSideAdjustment(-engine.commonSpeedParameters.maxSwingSideStretch, engine.commonSpeedParameters.maxSwingSideStretch);
    sideSwingStrecht += engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime * swingSign;
    sideSwingStrecht = maxSideAdjustment.limit(sideSwingStrecht);
  }
  sideSwing += sideSwingStrecht;

  // Forward translation
  forwardSupport = forwardSupport0 + (-useForwardStep * 0.5f - forwardSupport0) * Rangef::ZeroOneRange().limit(ratio / duration);
  forwardSwing = forwardSwing0 + (useForwardStep * 0.5f - forwardSwing0) * parabolicStep(ratio, duration); // swing-foot follow-through

  // Rotation
  turnVal = turnRL0 + (swingSign * useTurnStep * 0.5f - turnRL0) * parabolicStep(ratio, duration);

  // Feet height
  const Rangef range(-engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime, engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeSlow(-engine.commonSpeedParameters.slowFeetAdjustment * engine.motionCycleTime, engine.commonSpeedParameters.slowFeetAdjustment * engine.motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeAfterKick(-engine.commonSpeedParameters.afterKickFeetHeightAdjustment * engine.motionCycleTime, engine.commonSpeedParameters.afterKickFeetHeightAdjustment * engine.motionCycleTime);
  // Support foot
  if(footHeightSupport0 < 0.f)
    footHeightSupport0 -= rangeSlow.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  else
    footHeightSupport0 -= useSlowSupportFootHeightAfterKickInterpolation ? rangeAfterKick.limit(footHeightSupport0) : range.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  footHeightSupport = footHeightSupport0;

  // Swing foot
  const float oldFootHeightSwing = maxFootHeight * parabolicFootHeight(ratioBase - engine.motionCycleTime, heightDuration);
  footHeightSwing = maxFootHeight * parabolicFootHeight(ratioBase, heightDuration); // lift swing foot

  if(ratioBase > duration + engine.commonSpeedParameters.swingStretchAfterOvertime && !engine.theSolePressureState.legInfo[isLeftPhase ? Legs::left : Legs::right].hasPressure && footHeightSwing0 >= -10.f)
    footHeightSwing0 = std::max(engine.commonSpeedParameters.maxSwingStretchLength, footHeightSwing0 - engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime);
  else
  {
    const float swingChange = footHeightSwing - oldFootHeightSwing;
    const float swing0Change = Rangef(std::min(0.f, swingChange), std::max(0.f, swingChange)).limit(footHeightSwing0);
    footHeightSwing0 -= swing0Change; // return swing foot offset to 0
    if(footHeightSwing0 < 0 || footHeightSwing + footHeightSwing0 > maxFootHeight) // in case height is outside the allowed boundary, reduce it based in the max allowed velocity
      footHeightSwing0 -= Rangef(range.min + std::max(0.f, -swing0Change), range.max + std::min(0.f, -swing0Change)).limit(footHeightSwing0);
  }

  footHeightSwing += footHeightSwing0;
}

void WalkPhase::calcWalkKickFootOffsets(const float swingSign, const float ratio,
                                        float& forwardSwing0, float& forwardSupport0, float& forwardSwing,
                                        float& forwardSupport, float& sideSwing0, float& sideSupport0,
                                        float& sideSwing, float& sideSwingStrecht, float& sideSupport, float& footHeightSwing0,
                                        float& footHeightSupport0, float& footHeightSwing,
                                        float& footHeightSupport, Angle& turnVal, WalkKickStep& walkKickStep,
                                        Angle& turnRL0)
{
  // TODO move upwards and apply turn reduction
  // Only update if no delay phase was applied
  if(!walkKickStep.usedWalkDelay && engine.theWalkKickGenerator.dynamicWalkKickStepUpdate(walkKickStep, ratio) &&
     engine.configuredParameters.dynamicKickStepUpdate)
  {
    // Keyframes changed, update them once again
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
  }
  // Get current keyframe index and duration
  int keyframeIndex = 0;
  float finishedDuration = 0.f;
  float finishedRatio = 0.f;
  for(size_t i = 0; i < walkKickStep.keyframe.size(); i++)
  {
    if(walkKickStep.keyframe[i].reachedWaitPosition)
    {
      finishedDuration += stepDuration * walkKickStep.keyframe[i].stepRatio;
      finishedRatio += walkKickStep.keyframe[i].stepRatio;
      keyframeIndex++;
    }
    // Drawings
    engine.theWalkKickGenerator.drawStep(walkKickStep.keyframe[i].stepTargetSwing);
  }
  float useDuration = stepDuration * walkKickStep.keyframe[keyframeIndex].stepRatio;
  float useRatio = ratio - finishedDuration;

  // Next step target started
  if(useRatio / useDuration >= 1.f && keyframeIndex + 1 < static_cast<int>(walkKickStep.keyframe.size()))
  {
    walkKickStep.keyframe[keyframeIndex].reachedWaitPosition = true;
    finishedDuration += stepDuration * walkKickStep.keyframe[keyframeIndex].stepRatio;
    keyframeIndex++;
    useDuration = stepDuration * walkKickStep.keyframe[keyframeIndex].stepRatio;
    useRatio = ratio - finishedDuration;
    if(ratio != engine.motionCycleTime)
    {
      if(!walkKickStep.useLastKeyframeForSupportFootXTranslation)
        forwardSupport0 = forwardSupport; // only if the keyframe interpolation is active
      forwardSwing0 = forwardSwing;
      sideSwing0 = sideSwing;
      sideSupport0 = sideSupport;
      turnRL0 = turnVal;
    }
  }

  // Get current step targets and other parameters
  float useSideHipShiftRatio = finishedRatio + walkKickStep.keyframe[keyframeIndex].stepRatio;
  const Angle useTurnStep = walkKickStep.keyframe[keyframeIndex].stepTargetConverted.rotation;
  const float useForwardStep = walkKickStep.keyframe[keyframeIndex].stepTargetConverted.translation.x();
  const float useForwardSupportStep = !walkKickStep.useLastKeyframeForSupportFootXTranslation ? useForwardStep : walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.translation.x();
  const float useSideStep = walkKickStep.keyframe[keyframeIndex].stepTargetConverted.translation.y();
  const Vector2f swingSpeedUp = walkKickStep.keyframe[keyframeIndex].speedUpSwing;
  const Vector2f offsetSwingFoot = walkKickStep.keyframe[keyframeIndex].stepTargetSwingConverted.translation - walkKickStep.keyframe[keyframeIndex].stepTargetConverted.translation;
  const WalkKickStep::InterpolationType interpolationType = walkKickStep.keyframe[keyframeIndex].interpolationType;
  const float useForwardSupportRatio = !walkKickStep.useLastKeyframeForSupportFootXTranslation ? useRatio : ratio;
  const float useForwardSupportDuration = !walkKickStep.useLastKeyframeForSupportFootXTranslation ? useDuration : stepDuration;

  const float targetLastSideHipShift = walkKickStep.leftoverSideHipShift * (1.f - useSideHipShiftRatio);
  sideSupport = sideSupport0 + ((-useSideStep + useSideHipShiftRatio * sideHipShift + targetLastSideHipShift) * engine.kinematicParameters.sidewaysHipShiftFactor - sideSupport0) * Rangef::ZeroOneRange().limit(useRatio / useDuration);
  sideSwing = sideSwing0 + ((useSideStep + useSideHipShiftRatio * sideHipShift + targetLastSideHipShift) * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor) + offsetSwingFoot.y() - sideSwing0) * swingInterpolation(useRatio * swingSpeedUp.y(), useDuration, interpolationType);

  if(ratio > stepDuration + engine.commonSpeedParameters.swingStretchAfterOvertime && !engine.theSolePressureState.legInfo[isLeftPhase ? Legs::left : Legs::right].hasPressure)
  {
    sideSwingStrecht += engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime * swingSign;
    if(swingSign > 0.f)
      sideSwingStrecht = std::min(sideSwingStrecht, engine.commonSpeedParameters.maxSwingSideStretch);
    else
      sideSwingStrecht = std::max(sideSwingStrecht, -engine.commonSpeedParameters.maxSwingSideStretch);
  }
  sideSwing += sideSwingStrecht;

  if(!walkKickStep.keyframe[keyframeIndex].holdXSupportTarget)
    forwardSupport = forwardSupport0 + (-useForwardSupportStep * 0.5f - forwardSupport0) * Rangef::ZeroOneRange().limit(useForwardSupportRatio / useForwardSupportDuration);
  else
    forwardSupport = forwardSupport0;

  if(!walkKickStep.keyframe[keyframeIndex].holdXSwingTarget)
    forwardSwing = forwardSwing0 + (useForwardStep * 0.5f + offsetSwingFoot.x() - forwardSwing0) * swingInterpolation(useRatio * swingSpeedUp.x(), useDuration, interpolationType); // swing-foot follow-through
  else
    forwardSwing = forwardSwing0;

  turnVal = turnRL0 + (swingSign * useTurnStep * 0.5f - turnRL0) * parabolicStep(useRatio, useDuration);

  // Feet height
  const Rangef range(-engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime, engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeSlow(-engine.commonSpeedParameters.slowFeetAdjustment * engine.motionCycleTime, engine.commonSpeedParameters.slowFeetAdjustment * engine.motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeAfterKick(-engine.commonSpeedParameters.afterKickFeetHeightAdjustment * engine.motionCycleTime, engine.commonSpeedParameters.afterKickFeetHeightAdjustment * engine.motionCycleTime);
  if(footHeightSupport0 < 0.f)
    footHeightSupport0 -= rangeSlow.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  else
    footHeightSupport0 -= useSlowSupportFootHeightAfterKickInterpolation ? rangeAfterKick.limit(footHeightSupport0) : range.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  footHeightSupport = footHeightSupport0;

  footHeightSwing = maxFootHeight * parabolicFootHeight(ratio, stepHeightDuration); // lift swing foot
  footHeightSwing0 -= range.limit(footHeightSwing0); // return swing foot offset to 0
  footHeightSwing += footHeightSwing0;
  footHeightSwing = ratio / stepHeightDuration > 0.5f ? std::max(footHeightSwing, walkKickStep.reduceSwingFootHeight) : footHeightSwing;
}

void WalkingEngine::calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR,
                                  const float footHL, const float footHR, const Angle turn, const float currentWalkHipHeight,
                                  const Angle soleRotationYL, const Angle soleRotationXL, const Angle soleRotationYR, const Angle soleRotationXR,
                                  Pose3f& leftFoot, Pose3f& rightFoot) const
{
  ASSERT(kinematicParameters.walkHipHeight.isInside(currentWalkHipHeight));

  leftFoot = Pose3f(0, theRobotDimensions.yHipOffset + kinematicParameters.yHipOffset, 0)
             .translate(forwardL - kinematicParameters.torsoOffset, sideL, -(currentWalkHipHeight - footHL))
             .rotateZ(turn)
             .rotateY(soleRotationYL)
             .rotateX(soleRotationXL);
  rightFoot = Pose3f(0, -theRobotDimensions.yHipOffset - kinematicParameters.yHipOffset, 0)
              .translate(forwardR - kinematicParameters.torsoOffset, sideR, -(currentWalkHipHeight - footHR))
              .rotateZ(-turn)
              .rotateY(soleRotationYR)
              .rotateX(soleRotationXR);
}

void WalkPhase::setArms(Pose3f& leftFoot, Pose3f rightFoot, JointRequest& jointRequest, JointRequest& walkArms)
{
  walkArms.angles[Joints::lShoulderPitch] = engine.armParameters.armShoulderPitch + (leftFoot.translation.x() + engine.kinematicParameters.torsoOffset + armPitchShift[Legs::left]) * engine.armParameters.armShoulderPitchFactor / 1000.f;
  walkArms.angles[Joints::lShoulderRoll] = engine.armParameters.armShoulderRoll + std::abs(leftFoot.translation.y() - engine.theRobotDimensions.yHipOffset - engine.kinematicParameters.yHipOffset) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  walkArms.angles[Joints::lElbowYaw] = engine.armParameters.armElbowYaw;
  walkArms.angles[Joints::lElbowRoll] = 0_deg;
  walkArms.angles[Joints::lWristYaw] = -90_deg;
  walkArms.angles[Joints::lHand] = 0.f;
  walkArms.angles[Joints::rShoulderPitch] = engine.armParameters.armShoulderPitch + (rightFoot.translation.x() + engine.kinematicParameters.torsoOffset + armPitchShift[Legs::right]) * engine.armParameters.armShoulderPitchFactor / 1000.f;
  walkArms.angles[Joints::rShoulderRoll] = -engine.armParameters.armShoulderRoll - std::abs(rightFoot.translation.y() + engine.theRobotDimensions.yHipOffset + engine.kinematicParameters.yHipOffset) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  walkArms.angles[Joints::rElbowYaw] = -engine.armParameters.armElbowYaw;
  walkArms.angles[Joints::rElbowRoll] = 0_deg;
  walkArms.angles[Joints::rWristYaw] = 90_deg;
  walkArms.angles[Joints::rHand] = 0.f;

  // Override ignore arm joints
  if(jointRequest.angles[Joints::lShoulderPitch] != JointAngles::ignore)
  {
    MotionUtilities::copy(jointRequest, leftArm, engine.theStiffnessSettings, Joints::firstLeftArmJoint, Joints::firstRightArmJoint);
    leftArmInterpolationStart = engine.theFrameInfo.time;
    leftArmInterpolationTime = std::max(std::abs(leftArm.angles[Joints::lShoulderPitch] - walkArms.angles[Joints::lShoulderPitch]), std::abs(leftArm.angles[Joints::lShoulderRoll] - walkArms.angles[Joints::lShoulderRoll])) / engine.standInterpolationVelocity * 1000.f;
  }
  if(jointRequest.angles[Joints::rShoulderPitch] != JointAngles::ignore)
  {
    MotionUtilities::copy(jointRequest, rightArm, engine.theStiffnessSettings, Joints::firstRightArmJoint, Joints::firstLegJoint);
    rightArmInterpolationStart = engine.theFrameInfo.time;
    rightArmInterpolationTime = std::max(std::abs(rightArm.angles[Joints::rShoulderPitch] - walkArms.angles[Joints::rShoulderPitch]), std::abs(rightArm.angles[Joints::rShoulderRoll] - walkArms.angles[Joints::rShoulderRoll])) / engine.standInterpolationVelocity * 1000.f;
  }

  // Interpolate left arm
  if(jointRequest.angles[Joints::lShoulderPitch] == JointAngles::ignore && leftArmInterpolationTime > 0)
  {
    for(int joint = Joints::firstLeftArmJoint; joint < Joints::firstRightArmJoint; ++joint)
    {
      const float ratio = std::min(1.f, engine.theFrameInfo.getTimeSince(leftArmInterpolationStart) / leftArmInterpolationTime);
      jointRequest.angles[joint] = leftArm.angles[joint];
      jointRequest.angles[joint] += ratio * (walkArms.angles[joint] - leftArm.angles[joint]);
      jointRequest.stiffnessData.stiffnesses[joint] = StiffnessData::useDefault;
    }
  }
  // Interpolate right arm
  if(jointRequest.angles[Joints::rShoulderPitch] == JointAngles::ignore && rightArmInterpolationTime > 0)
  {
    for(int joint = Joints::firstRightArmJoint; joint < Joints::firstLegJoint; ++joint)
    {
      const float ratio = std::min(1.f, engine.theFrameInfo.getTimeSince(rightArmInterpolationStart) / rightArmInterpolationTime);
      jointRequest.angles[joint] = rightArm.angles[joint];
      jointRequest.angles[joint] += ratio * (walkArms.angles[joint] - rightArm.angles[joint]);
      jointRequest.stiffnessData.stiffnesses[joint] = StiffnessData::useDefault;
    }
  }
}

void WalkPhase::compensateArms(JointRequest& jointRequest, const JointRequest& walkArms)
{
  JointRequest temp = walkArms;
  for(int joint = 0; joint < Joints::firstArmJoint; ++joint)
    temp.angles[joint] = temp.angles[joint] != JointRequest::off && temp.angles[joint] != JointRequest::ignore
                         ? temp.angles[joint] : engine.theJointAngles.angles[joint];
  temp.angles[Joints::headPitch] = 0_deg;
  temp.angles[Joints::headYaw] = 0_deg;
  const RobotModel withWalkGeneratorArms(temp, engine.theRobotDimensions, engine.theMassCalibration); // walk arms
  for(int joint = 0; joint < Joints::firstLegJoint; ++joint) // with arm engine request and head request
    temp.angles[joint] = jointRequest.angles[joint] != JointRequest::off && jointRequest.angles[joint] != JointRequest::ignore
                         ? jointRequest.angles[joint] : engine.theJointAngles.angles[joint];
  RobotModel balanced(temp, engine.theRobotDimensions, engine.theMassCalibration);

  armCompensationAfterKick = std::max(0.f, armCompensationAfterKick - engine.motionCycleTime * 1000.f / engine.kinematicParameters.baseWalkPeriod);
  armCompensationTilt.push_front((balanced.centerOfMass.x() - withWalkGeneratorArms.centerOfMass.x()) * (1.f - armCompensationAfterKick) * engine.armParameters.comTiltFactor);

  PLOT("module:WalkingEngine:armCompensation", armCompensationTilt[0].toDegrees());
}

void WalkPhase::calculateSupportSideCompensation()
{
  if(!engine.supportCompensationParameters.isActive || walkState == standing || engine.theSolePressureState.legInfo[isLeftPhase ? Legs::left : Legs::right].hasPressure || walkKickStep.currentKick == WalkKicks::none || walkKickStep.longKickParams.size() == 0)
    return;

  JointRequest lastExecutedRequest;
  FOREACH_ENUM(Joints::Joint, joint)
    lastExecutedRequest.angles[joint] = engine.theJointPlay.jointState[joint].lastExecutedRequest;

  const RobotModel lastExecutedModel(lastExecutedRequest, engine.theRobotDimensions, engine.theMassCalibration);

  const Pose3f& measuredSupportSole = isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft;
  const Pose3f& requestedSupportSole = isLeftPhase ? lastExecutedModel.soleRight : lastExecutedModel.soleLeft;
  float forwardError = (measuredSupportSole.translation.x() - requestedSupportSole.translation.x()) * engine.supportCompensationParameters.scaling;
  forwardError *= mapToRange(tBase / stepDuration, engine.supportCompensationParameters.timeScaling.min, engine.supportCompensationParameters.timeScaling.max, 0.f, 1.f);
  supportForwardCompensation.update(forwardError);
}
