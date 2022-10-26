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
#include "Tools/Motion/ForwardKinematic.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Streaming/InStreams.h"
#include <cmath>

MAKE_MODULE(WalkingEngine, motionControl);

WalkingEngine::WalkingEngine()
{
  InMapFile stream("walkingEngineCommon.cfg");
  if(stream.exists())
    stream >> static_cast<WalkingEngineCommon&>(*this);

  const DummyPhase dummy(MotionPhase::playDead);
  WalkPhase phase(*this, Pose2f(), dummy);

  translationPolygon = phase.getTranslationPolygon(speedParameters.maxSpeedBackwards, speedParameters.maxSpeed.translation.x(), speedParameters.maxSpeed.translation.y());
  std::vector<Vector2f> translationPolygonTemp = phase.getTranslationPolygon(speedParameters.maxSpeed.translation.x(), speedParameters.maxSpeed.translation.x(), speedParameters.maxSpeed.translation.y());

  filterTranslationPolygon(translationPolygonAfterKick, translationPolygonTemp, translationPolygonTemp);

  // Set the masses of all leg joints to 0
  lightMassCalibration = theMassCalibration;
  for(size_t i = Limbs::firstLeftLegLimb; i < Limbs::torso; i++)
    lightMassCalibration.masses[i].mass = 0.f;
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
  walkStepData.updateWalkValues = [&walkStepData](const Pose2f& stepTarget, const float stepDuration)
  {
    walkStepData.stepTarget = stepTarget;
    walkStepData.stepDuration = stepDuration;
  };
}

void WalkingEngine::update(WalkGenerator& walkGenerator)
{
  MODIFY("parameters:WalkingEngine:common", static_cast<WalkingEngineCommon&>(*this));
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:footMid", "robot");
  DECLARE_DEBUG_DRAWING3D("module:WalkingEngine:balance", "robot");
  DECLARE_DEBUG_DRAWING3D("module:WalkStepAdjustment:footMid", "robot");
  DECLARE_DEBUG_DRAWING3D("module:WalkStepAdjustment:balance", "robot");
  DECLARE_PLOT("module:WalkingEngine:leftFootHightCurrent");
  DECLARE_PLOT("module:WalkingEngine:leftFootHightDesired");
  DECLARE_PLOT("module:WalkingEngine:rightFootHightCurrent");
  DECLARE_PLOT("module:WalkingEngine:rightFootHightDesired");
  DECLARE_PLOT("module:WalkingEngine:leftFootForwardCurrent");
  DECLARE_PLOT("module:WalkingEngine:leftFootForwardDesired");
  DECLARE_PLOT("module:WalkingEngine:rightFootForwardCurrent");
  DECLARE_PLOT("module:WalkingEngine:rightFootForwardDesired");
  DECLARE_PLOT("module:WalkingEngine:leftFootSideCurrent");
  DECLARE_PLOT("module:WalkingEngine:leftFootSideDesired");
  DECLARE_PLOT("module:WalkingEngine:rightFootSideCurrent");
  DECLARE_PLOT("module:WalkingEngine:rightFootSideDesired");
  DECLARE_PLOT("module:WalkingEngine:armCompensation");
  DECLARE_PLOT("module:WalkingEngine:Data:leftAdjustment");
  DECLARE_PLOT("module:WalkingEngine:Data:rightAdjustment");
  DECLARE_PLOT("module:WalkStepAdjustment:Data:leftAdjustment");
  DECLARE_PLOT("module:WalkStepAdjustment:Data:rightAdjustment");
  DECLARE_PLOT("module:WalkingEngine:SpeedPolygon:front");
  DECLARE_PLOT("module:WalkingEngine:SpeedPolygon:back");
  DECLARE_PLOT("module:WalkingEngine:Data:sideBalance");
  DECLARE_PLOT("module:WalkingEngine:Data:refCOMY");
  DECLARE_PLOT("module:WalkingEngine:Data:sagital");
  DECLARE_PLOT("module:WalkingEngine:Data:lateral");
  DECLARE_PLOT("module:WalkingEngine:Data:hipBalancing");
  DECLARE_PLOT("module:WalkingEngine:Data:kneeHipBalance");
  DECLARE_PLOT("module:WalkingEngine:Data:supportSoleRotationY");
  DECLARE_PLOT("module:WalkingEngine:Data:supportSoleRotationCompensationY");
  DECLARE_PLOT("module:WalkingEngine:Data:feetHeightDifference");
  DECLARE_DEBUG_RESPONSE("module:WalkingEngine:feetPositions");

  filteredGyroX = balanceParameters.gyroLowPassRatio * filteredGyroX + (1.f - balanceParameters.gyroLowPassRatio) * theInertialData.gyro.x();
  filteredGyroY = balanceParameters.gyroLowPassRatio * filteredGyroY + (1.f - balanceParameters.gyroLowPassRatio) * theInertialData.gyro.y();

  // Decide if the gyroBalanceFactors shall be taken from the WalkLearner.
  if(theWalkModifier.numOfGyroPeaks > 0)
  {
    if(theWalkLearner.newGyroBackwardBalance > 0)
      balanceParameters.gyroBackwardBalanceFactor = theWalkLearner.newGyroBackwardBalance;
    if(theWalkLearner.newGyroForwardBalance > 0)
      balanceParameters.gyroForwardBalanceFactor = theWalkLearner.newGyroForwardBalance;
  }

  // Set walk functions
  walkGenerator.createPhase = [this](const Pose2f& step, const MotionPhase& lastPhase)
  {
    return std::make_unique<WalkPhase>(*this, step, lastPhase);
  };

  walkGenerator.createPhaseWithNextPhase = [this](const WalkKickStep& walkKickStep, const MotionPhase& lastPhase, const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback)
  {
    ASSERT(walkKickStep.keyframe.size() > 0);
    const Pose2f& goalStepTarget = walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTarget;
    return std::make_unique<WalkPhase>(*this, goalStepTarget, lastPhase, createNextPhaseCallback, walkKickStep);
  };

  walkGenerator.isNextLeftPhase = [this](const bool shouldBeLeft, const MotionPhase& lastPhase)
  {
    // TODO: This function should be more correct.
    if(lastPhase.type != MotionPhase::walk && lastPhase.type != MotionPhase::kick)
      return shouldBeLeft;
    // Check if the foot support switch was a predicted one
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      if(!lastWalkPhase.supportSwitchInfo.isFootSupportSwitch)
        return theFootSupport.support > 0.f;
    }
    return theFootSupport.support < 0.f;
  };

  walkGenerator.wasLastPhaseLeftPhase = [this](const MotionPhase& lastPhase)
  {
    if(lastPhase.type != MotionPhase::walk)
      return theFootSupport.support < 0.f;

    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    return lastWalkPhase.isLeftPhase;
  };

  walkGenerator.calcNextStepDuration = [this](const Pose2f& velocity)
  {
    return stepDurationSpeedTarget(velocity.translation.y());
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

  walkGenerator.getRotationRange = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio)
  {
    const float useRobotStateFactor = useJointPlayScaling ? theJointPlay.qualityOfRobotHardware : 1.f;
    const Angle rotation = speedParameters.maxSpeed.rotation * useRobotStateFactor + speedParameters.minSpeed.rotation * (1.f - useRobotStateFactor);
    const float innerTurn = 2.f * stepSizeParameters.insideTurnRatio * rotation * std::abs(walkSpeedRatio.rotation) * kinematicParameters.baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - stepSizeParameters.insideTurnRatio) * rotation * std::abs(walkSpeedRatio.rotation) * kinematicParameters.baseWalkPeriod / 1000.f;
    return Rangea(isLeftPhase ? -innerTurn : -outerTurn, isLeftPhase ? outerTurn : innerTurn);
  };

  walkGenerator.getTranslationPolygon = [this](const bool isLeftPhase, float rotation, const MotionPhase& lastPhase, const Pose2f& walkSpeedRatio, std::vector<Vector2f>& translationPolygon, std::vector<Vector2f>& translationPolygonNoCenter, const bool fastWalk)
  {
    bool useFastWalk = fastWalk;
    // After an InWalkKick, the next steps are balance steps to ensure that the robot will not fall
    bool overrideForward = false;
    Vector2f forwardBalance = Vector2f(0.f, 0.f);
    Pose2f useWalkSpeedRatio = walkSpeedRatio;
    bool armsAreBack = false;
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      if(lastWalkPhase.walkStepAdjustment.reduceWalkingSpeed > 0)
        useWalkSpeedRatio.translation.x() *= speedParameters.walkSpeedReductionFactor;
      if(lastWalkPhase.noFastTranslationPolygonSteps > 0)
        useFastWalk = false;
      if(lastWalkPhase.doBalanceSteps > 0)
      {
        const Pose3f& supportInTorso = !isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight;
        const Pose3f& swingInTorso = isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight;
        const Pose3f currentComInFloor = (theTorsoMatrix * supportInTorso).inverse() * (theTorsoMatrix * theRobotModel.centerOfMass);

        const float swingXRequested = (isLeftPhase ? lastWalkPhase.forwardL : lastWalkPhase.forwardR) + (isLeftPhase ? lastWalkPhase.walkStepAdjustment.lastLeftAdjustmentX : lastWalkPhase.walkStepAdjustment.lastRightAdjustmentX);
        const float currentComToSwingX = (theTorsoMatrix * theRobotModel.centerOfMass).x() - swingXRequested;

        const Pose3f& ref = (lastPhase.type == MotionPhase::walk ? static_cast<const WalkPhase&>(lastPhase).lastComInFoot : currentComInFloor);

        const float comMovementCausedByFeet = std::min(0.f, - (supportInTorso.translation.x() + swingInTorso.translation.x()));
        const float diff = currentComInFloor.translation.x() - ref.translation.x();

        const float predictedCom = currentComInFloor.translation.x() + diff + comMovementCausedByFeet;

        overrideForward = true;
        forwardBalance = Vector2f(std::min(currentComToSwingX + comMovementCausedByFeet, predictedCom) - 20.f, std::max(currentComToSwingX + comMovementCausedByFeet + std::min(0.f, predictedCom / 2.f), predictedCom) - 20.f);
        if(forwardBalance.y() > 0.f) // Force back walking, but when only forward steps are allowed, allow stopping
          forwardBalance.x() = std::min(forwardBalance.x(), -0.01f);
      }
      armsAreBack = theFrameInfo.getTimeSince(lastWalkPhase.leftArmInterpolationStart) < Constants::motionCycleTime * 2.f * 1000.f || theFrameInfo.getTimeSince(lastWalkPhase.leftArmInterpolationStart) < Constants::motionCycleTime * 2.f * 1000.f;
    }

    Vector2f frontLeft, backRight;
    const Pose2f lastStep = lastPhase.type == MotionPhase::walk ? static_cast<const WalkPhase&>(lastPhase).step : Pose2f();
    const Vector2f maxStepSizeChange = speedParameters.maxAcceleration * kinematicParameters.baseWalkPeriod / 1000.f;
    const Vector2f maxStepSizeChangeToZero = speedParameters.maxDeceleration * kinematicParameters.baseWalkPeriod / 1000.f;
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

    // TODO: Limit sidewards acceleration.
    backRight.y() = -1000.f;
    frontLeft.y() = 1000.f;

    // Scale by JointPlay
    const float useRobotStateFactor = useJointPlayScaling ? theJointPlay.qualityOfRobotHardware : 1.f;
    const Vector2f useSpeed = speedParameters.maxSpeed.translation * useRobotStateFactor + speedParameters.minSpeed.translation * (1.f - useRobotStateFactor);
    const float useSpeedBackwards = speedParameters.maxSpeedBackwards * useRobotStateFactor + speedParameters.minSpeedBackwards * (1.f - useRobotStateFactor);

    // Limit to maximum speed (which is influenced by the rotation).
    const Angle useReduceFactor = armsAreBack ? 10_deg : stepSizeParameters.reduceTranslationFromRotation.x();
    const float tFactorX = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceFactor) / (stepSizeParameters.noTranslationFromRotation.x() - useReduceFactor))));
    const float reduceTranstionY = !useFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : ((isLeftPhase && rotation > 0_deg) || (!isLeftPhase && rotation < 0_deg) ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
    const float useReduceTranslationFromRotation = !useFastWalk ? stepSizeParameters.reduceTranslationFromRotation.y() : stepSizeParameters.reduceTranslationYFromRotationFast;
    const float tFactorY = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceTranslationFromRotation) / (reduceTranstionY - useReduceTranslationFromRotation))));
    backRight.x() = std::max(backRight.x(), tFactorX * -useSpeedBackwards * kinematicParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()));
    backRight.y() = std::max(backRight.y(), tFactorY * -2.f * useSpeed.y() * kinematicParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.y()));
    frontLeft.x() = std::min(frontLeft.x(), tFactorX * useSpeed.x() * kinematicParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()));
    frontLeft.y() = std::min(frontLeft.y(), tFactorY * 2.f * useSpeed.y() * kinematicParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.y()));

    if(useFastWalk)
    {
      const float useMinXBackwardTranslation = stepSizeParameters.minXBackwardTranslationFastRange.min * (1.f - useRobotStateFactor) + stepSizeParameters.minXBackwardTranslationFastRange.max * useRobotStateFactor;
      frontLeft.x() = std::max(stepSizeParameters.minXForwardTranslationFast * useWalkSpeedRatio.translation.x(), frontLeft.x());
      backRight.x() = std::min(useMinXBackwardTranslation * useWalkSpeedRatio.translation.x(), backRight.x());
    }

    // Step size in x translation has a min size
    const float maxMinStepX = std::min(stepSizeParameters.minXTranslationStep, (useWalkSpeedRatio.translation.x() >= 0.f ? useSpeed.x() : useSpeedBackwards) * kinematicParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()) + 0.01f);

    if(!overrideForward)
    {
      backRight.x() = std::min(backRight.x(), -maxMinStepX);
      frontLeft.x() = std::max(frontLeft.x(), maxMinStepX);
    }

    // (0,0) must be part of the rectangle.
    backRight.x() = std::min(backRight.x(), -.01f);
    frontLeft.x() = std::max(frontLeft.x(), .01f);
    backRight.y() = std::min(backRight.y(), -.01f);
    frontLeft.y() = std::max(frontLeft.y(), .01f);

    Vector2f frontLeftNoCenter = frontLeft;
    Vector2f backRightNoCenter = backRight;
    if(overrideForward)
    {
      frontLeftNoCenter.x() = forwardBalance.y();
      backRightNoCenter.x() = forwardBalance.x();
      frontLeftNoCenter.y() = 30.f;
      backRightNoCenter.y() = -30.f;
      backRight.y() = std::max(backRight.y(), -30.f);
      frontLeft.y() = std::min(frontLeft.y(), 30.f);
    }

    generateTranslationPolygon(translationPolygon, backRight, frontLeft);
    generateTranslationPolygon(translationPolygonNoCenter, backRightNoCenter, frontLeftNoCenter);
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
      left.translate(lastRobotModel.soleLeft.translation.head<2>() + Vector2f(0.f, kinematicParameters.torsoOffset)).rotate(lastRobotModel.soleLeft.rotation.getZAngle());
      right.translate(lastRobotModel.soleRight.translation.head<2>() + Vector2f(0.f, kinematicParameters.torsoOffset)).rotate(lastRobotModel.soleRight.rotation.getZAngle());
    }
    return std::make_tuple(left, right, lastStep);
  };

  walkGenerator.getLastStepChange = [](const MotionPhase& lastPhase)
  {
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
      Pose2f stepChange;
      stepChange.rotation = std::abs(lastWalkPhase.turnRL0 - lastWalkPhase.turnRL);
      stepChange.translation.x() = std::abs(lastWalkPhase.forwardR0 - lastWalkPhase.forwardR - (lastWalkPhase.forwardL0 - lastWalkPhase.forwardL)) * 0.5f;
      stepChange.translation.y() = std::abs(lastWalkPhase.sideR0 - lastWalkPhase.sideR - (lastWalkPhase.sideL0 - lastWalkPhase.sideL));
      return stepChange;
    }
    return Pose2f();
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

void WalkingEngine::filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal)
{
  // adjust y forward
  Geometry::Line lineForwardAdjusted(polygonIn[1], (polygonIn[1] - polygonIn[2]).normalized());
  Vector2f leftY;
  if(Geometry::getIntersectionOfLineAndConvexPolygon(polygonOriginal, lineForwardAdjusted, leftY))
  {
    polygonIn[1].y() = std::min(leftY.y(), polygonIn[0].y());
    polygonIn[2].y() = std::max(-leftY.y(), polygonIn[3].y());
  }

  // adjust y backward
  Geometry::Line lineBackAdjusted(polygonIn[5], (polygonIn[6] - polygonIn[5]).normalized());
  if(Geometry::getIntersectionOfLineAndConvexPolygon(polygonOriginal, lineBackAdjusted, leftY))
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

void WalkingEngine::generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft)
{
  std::vector<Vector2f> translationPolygonTemp = this->translationPolygon;
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
    translationPolygonTemp[0].x() = translationPolygonTemp[1].x() = std::max(this->translationPolygon[7].x(), translationPolygonTemp[0].x());
    translationPolygonTemp[2].x() = translationPolygonTemp[3].x() = std::max(this->translationPolygon[4].x(), translationPolygonTemp[3].x());
  }
  if(backRight.x() > 0.f)
  {
    translationPolygonTemp[7].x() = translationPolygonTemp[6].x() = std::min(this->translationPolygon[0].x(), translationPolygonTemp[7].x());
    translationPolygonTemp[4].x() = translationPolygonTemp[5].x() = std::min(this->translationPolygon[3].x(), translationPolygonTemp[4].x());
  }

  filterTranslationPolygon(polygon, translationPolygonTemp, this->translationPolygon);
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
  useSpeed.rotation = speedParameters.maxSpeed.rotation * useRobotStateFactor + speedParameters.minSpeed.rotation * (1.f - useRobotStateFactor);
  useSpeed.translation = speedParameters.maxSpeed.translation * useRobotStateFactor + speedParameters.minSpeed.translation * (1.f - useRobotStateFactor);
  const float useSpeedBackwards = speedParameters.maxSpeedBackwards * useRobotStateFactor + speedParameters.minSpeedBackwards * (1.f - useRobotStateFactor);

  walkingEngineOutput.maxSpeed = useSpeed;
  walkingEngineOutput.maxSpeedBackwards = useSpeedBackwards;
  walkingEngineOutput.walkStepDuration = kinematicParameters.baseWalkPeriod / 1000.f;
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

WalkPhase::WalkPhase(const WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase,
                     const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback,
                     const WalkKickStep& walkKickStep) :
  WalkPhaseBase(engine, walkKickStep),
  createNextPhaseCallback(createNextPhaseCallback)
{
  Pose2f useStepTarget = /*SystemCall::getMode() == SystemCall::logFileReplay ? engine.theWalkStepData.stepTarget : */stepTarget;
  if(lastPhase.type == MotionPhase::kick && useStepTarget == Pose2f())
    useStepTarget.translation.x() = 0.01f;
  const bool afterKickOrGetUp = (lastPhase.type == MotionPhase::kick || engine.theKeyframeMotionGenerator.wasLastGetUp(lastPhase) || lastPhase.type == MotionPhase::calibration);
  if(afterKickOrGetUp)
    timeWhenLastKick = engine.theFrameInfo.time;
  const bool standRequested = (useStepTarget.translation.x() == 0.f && useStepTarget.translation.y() == 0.f && useStepTarget.rotation == 0_deg) || (!engine.theGroundContactState.contact && !afterKickOrGetUp);
  timeWhenStandBegan = engine.theFrameInfo.time;
  engine.theEnergySaving.shutDown();
  leftArmInterpolationStart = engine.theFrameInfo.time;
  rightArmInterpolationStart = engine.theFrameInfo.time;
  leftArm = engine.theJointRequest;
  rightArm = engine.theJointRequest;
  Pose2f lastStepTarget;
  standInterpolationDuration = 1000.f; // default value

  leftArmInterpolationTime = rightArmInterpolationTime = engine.armParameters.armInterpolationTime;

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
    constructorAfterKickOrGetUpCase();

  // Previous phase was something else (e.g. playDead)
  else
    constructorOtherCase();

  // Previous was walking (or stopping). Calculate step size. Rotational requests must be compensated
  if(walkState != standing)
    constructorHelperInitWalkPhase(useStepTarget, lastStepTarget, afterKickOrGetUp, standRequested);
  else
    type = MotionPhase::stand;

  constructorWeightShiftStatus();

  // Prevent false time stamps when replaying logs
  if(engine.theFrameInfo.time < leftArmInterpolationStart)
    leftArmInterpolationStart = engine.theFrameInfo.time;
  if(engine.theFrameInfo.time < rightArmInterpolationStart)
    rightArmInterpolationStart = engine.theFrameInfo.time;

  // Decide if the gyroBalanceFactors shall be taken from the WalkLearner.
  if(engine.theWalkModifier.numOfGyroPeaks > 0)
    engine.theWalkLearner.setBaseWalkParams(engine.balanceParameters.gyroForwardBalanceFactor, engine.balanceParameters.gyroBackwardBalanceFactor, forwardStep);

  // Set walk values
  engine.theWalkStepData.updateWalkValues(step, stepDuration);

  lastComInFoot = (engine.theTorsoMatrix * (!isLeftPhase ? engine.theRobotModel.soleLeft : engine.theRobotModel.soleRight)).inverse() * (engine.theTorsoMatrix * engine.theRobotModel.centerOfMass);

  // Calc relative ball
  calculateBallPosition(isLeftPhase);

  if(walkState == stopping ||
     (std::abs(step.rotation) < engine.stoppingThresholds.rotation &&
      std::abs(step.translation.x()) < engine.stoppingThresholds.translation.x() &&
      std::abs(step.translation.y()) < engine.stoppingThresholds.translation.y()))
    stoppingCounter++;
  else
    stoppingCounter = 0;
}

void WalkPhase::update()
{
  tWalk += Constants::motionCycleTime * (1.f - engine.balanceParameters.slowdownFactor * Rangef::ZeroOneRange().limit((engine.theInertialData.angle.y() - armCompensationTilt + engine.balanceParameters.slowdownTorsoOffset + armCompensationTilt) / -engine.soleRotationParameter.minTorsoRotation));
  tBase += Constants::motionCycleTime;
  prevTurn = turnRL;
  lastPrevSideL = prevSideL;
  lastPrevSideR = prevSideR;
  prevSideL = sideL;
  prevSideR = sideR;
  const Angle useSoleRotationOffsetSpeed = engine.theFrameInfo.getTimeSince(timeWhenLastKick) > engine.speedParameters.soleRotationOffsetSpeedAfterKickTime ? this->engine.speedParameters.soleRotationOffsetSpeed.max : this->engine.speedParameters.soleRotationOffsetSpeed.min;
  lastWalkPhaseKneeHipBalance -= Rangea(-useSoleRotationOffsetSpeed * Constants::motionCycleTime, useSoleRotationOffsetSpeed * Constants::motionCycleTime).limit(lastWalkPhaseKneeHipBalance);

  const bool isSwitchAllowed = tBase > engine.supportSwitchPhaseRange.min * stepDuration;
  // enough time passed, prediction is allowed, prediction predicts an actual switch,
  const bool useFootSupportSwitchPrediction = std::abs(step.translation.angle()) < engine.maxWalkDirectionForFootPrediction && engine.useFootSupportSwitchPrediction;
  supportSwitchInfo.isPredictedSwitch = isSwitchAllowed && engine.theFootSupport.predictedSwitched && useFootSupportSwitchPrediction &&
                                        walkKickStep.currentKick == WalkKicks::none && isLeftPhase != engine.theFootSupport.support > 0.f && engine.theFootSupport.trustedSupport;
  const bool wrongSupportFoot = (engine.theFootSupport.support < 0.f != isLeftPhase);
  wrongSupportFootCounter += wrongSupportFoot && tBase > engine.minTimeForEarlySwitch && engine.theFootSupport.trustedSupport ? 1 : 0;
  supportSwitchInfo.isAbortSwitch = (wrongSupportFootCounter >= 3 || (wrongSupportFoot && earlySupportSwitchAllowed)) && walkKickStep.currentKick == WalkKicks::none;
  supportSwitchInfo.isFeetStepAbort = isSwitchAllowed && robotIsNotMoving >= engine.emergencyStep.emergencyNotMovingCounter;
  supportSwitchInfo.isNormalSwitch = isSwitchAllowed && engine.theFootSupport.switched;
  supportSwitchInfo.isOvertimeSwitch = tBase > engine.supportSwitchPhaseRange.max * stepDuration;
  supportSwitchInfo.isFootSupportSwitch = (supportSwitchInfo.isAbortSwitch || supportSwitchInfo.isFeetStepAbort || supportSwitchInfo.isNormalSwitch || supportSwitchInfo.isOvertimeSwitch) && !supportSwitchInfo.isPredictedSwitch;
}

bool WalkPhase::isDone(const MotionRequest& motionRequest) const
{
  return walkState == standing ? (motionRequest.motion != MotionRequest::stand && (!motionRequest.isWalking() || (engine.theGroundContactState.contact && standFactor == 0.f))) // switch from stand to walk
         : (supportSwitchInfo.isPredictedSwitch || supportSwitchInfo.isAbortSwitch || supportSwitchInfo.isNormalSwitch || supportSwitchInfo.isOvertimeSwitch || supportSwitchInfo.isFeetStepAbort);
}

void WalkPhase::calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  detectWalkingOnOpponentFeet();
  checkGyroState(motionRequest.motion == MotionRequest::stand && motionRequest.standHigh);
  if(walkState != standing)
  {
    // Walking
    if(walkKickStep.currentKick == WalkKicks::none)
    {
      if(isLeftPhase)
        calcFootOffsets(1.f, tWalk, tBase, stepDuration, stepHeightDuration, forwardL0, forwardR0, forwardL, forwardR, sideL0, sideR0, sideL, sideR, footHL0, footHR0, footHL, footHR, turnRL, forwardStep, sideStep, turnStep);
      else
        calcFootOffsets(-1.f, tWalk, tBase, stepDuration, stepHeightDuration, forwardR0, forwardL0, forwardR, forwardL, sideR0, sideL0, sideR, sideL, footHR0, footHL0, footHR, footHL, turnRL, forwardStep, sideStep, turnStep);
    }
    // InWalkKicks
    else
    {
      if(isLeftPhase)
        calcWalkKickFootOffsets(1.f, tBase, forwardL0, forwardR0, forwardL, forwardR, sideL0, sideR0, sideL, sideR, footHL0, footHR0, footHL, footHR, turnRL, walkKickStep, turnRL0);
      else
        calcWalkKickFootOffsets(-1.f, tBase, forwardR0, forwardL0, forwardR, forwardL, sideR0, sideL0, sideR, sideL, footHR0, footHL0, footHR, footHL, turnRL, walkKickStep, turnRL0);
    }
  }
  else
  {
    calcJointsHelperInterpolateStanding(motionRequest);
  }

  Pose3f leftFoot;
  Pose3f rightFoot;
  engine.calcFeetPoses(forwardL, forwardR, sideL, sideR, footHL, footHR, turnRL, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFoot, rightFoot); // current request

  VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);

  const bool applyEnergySavingStand = walkState == standing && standFactor == 0.f && engine.theFrameInfo.getTimeSince(timeWhenStandBegan) > engine.standStiffnessDelay && robotIsNotMoving >= engine.standHighNotMovingCounter;
  const bool applyEnergySavingHighStand = walkState == standing && standFactor == 1.f && engine.theFrameInfo.getTimeSince(timeWhenStandHighBegan) > engine.lowStiffnessDelay;

  // Leg stiffness
  const int legStiffness = applyEnergySavingHighStand ? engine.lowStiffnessLegs : (applyEnergySavingStand ? StiffnessData::useDefault : engine.walkStiffness);
  for(uint8_t i = Joints::firstLegJoint; i < Joints::numOfJoints; ++i)
    jointRequest.stiffnessData.stiffnesses[i] = legStiffness;

  // Different ankle pitch stiffness in standHigh
  if(applyEnergySavingHighStand)
  {
    jointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = engine.lowStiffnessAnklePitch;
    jointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = engine.lowStiffnessAnklePitch;
  }

  // Arm swing
  JointRequest walkArms = jointRequest;
  setArms(leftFoot, rightFoot, jointRequest, walkArms);

  // Compensate arm position
  compensateArms(leftFoot, rightFoot, jointRequest, walkArms);
  if(standFactor < 0.f)
  {
    VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);
    // Interpolate to stand.
    const float f = 0.5f * (std::sin((standFactor + 0.5f) * pi) - 1.f);
    for(int i = Joints::firstArmJoint; i < Joints::numOfJoints; ++i)
      if(jointRequest.angles[i] != JointAngles::off)
        jointRequest.angles[i] += f * (jointRequest.angles[i] - startJointAngles.angles[i]);
  }
  else
  {
    // InWalkKicks and balancing
    balanceFeetPoses(leftFoot, rightFoot);
    VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f(0.f, -armCompensationTilt), jointRequest, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);
  }

  // Rotate swing foot sole to prevent colliding with the ground
  calcJointsHelperFootSoleRotations(jointRequest, armCompensationTilt);
  // Add gyro balancing
  addGyroBalance(jointRequest);

  // Apply stand heat
  if(!(applyEnergySavingHighStand || applyEnergySavingStand) && engine.theEnergySaving.state == EnergySaving::EnergyState::working)
    engine.theEnergySaving.reset();
  if(!(applyEnergySavingHighStand || applyEnergySavingStand) && engine.theEnergySaving.state == EnergySaving::EnergyState::waiting)
    engine.theEnergySaving.shutDown();
  if(applyEnergySavingHighStand || applyEnergySavingStand || engine.theEnergySaving.state != EnergySaving::off)
    engine.theEnergySaving.applyHeatAdjustment(jointRequest, true, true, standFactor == 1.f, engine.theCalibrationRequest.preciseJointPositions);

  applyWalkKickLongKickOffset(jointRequest, tBase);

  odometryOffset = getOdometryOffset();

  motionInfo.isMotionStable = true;
  motionInfo.isWalkPhaseInWalkKick = walkKickStep.currentKick != WalkKicks::none;
  motionInfo.speed = stepDuration > 0.f ? Pose2f(turnStep / stepDuration, forwardStep / stepDuration, sideStep / stepDuration) : Pose2f(); // TODO

  debugDrawingFeetPositions(jointRequest);
}

std::unique_ptr<MotionPhase> WalkPhase::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  const bool standingPossible = isStandingPossible(forwardL, forwardR, lastPrevSideL, lastPrevSideR, turnRL, walkStepAdjustment.lastLeftAdjustmentX, walkStepAdjustment.lastRightAdjustmentX);
  const bool normalStopRequest = defaultNextPhase.type != MotionPhase::walk && walkState != stopping && walkState != standing && standingPossible;
  const bool standForKickRequest = defaultNextPhase.type == MotionPhase::kick && stoppingCounter < engine.stoppingCounterThreshold && normalStopRequest;

  // There must be another walk phase if the robot should transition into something other than walking and the feet are not next to each other.
  if(normalStopRequest || standForKickRequest)
    return std::make_unique<WalkPhase>(engine, standForKickRequest && stoppingCounter + 1 < engine.stoppingCounterThreshold ? Pose2f(0.f, 0.0001f, 0.f) : Pose2f(), *this);
  if(std::unique_ptr<MotionPhase> nextPhase; createNextPhaseCallback && (nextPhase = createNextPhaseCallback(*this)))
    return nextPhase;
  return std::unique_ptr<MotionPhase>();
}

void WalkPhase::calcFootOffsets(const float swingSign, const float ratio, const float ratioBase, const float duration, const float heightDuration,
                                const float forwardSwing0, const float forwardSupport0, float& forwardSwing,
                                float& forwardSupport, const float sideSwing0, const float sideSupport0,
                                float& sideSwing, float& sideSupport, float& footHeightSwing0,
                                float& footHeightSupport0, float& footHeightSwing,
                                float& footHeightSupport, Angle& turnVal,
                                const float useForwardStep, const float useSideStep, const Angle useTurnStep)
{
  // Side translation
  sideSupport = sideSupport0 + (-useSideStep * engine.kinematicParameters.sidewaysHipShiftFactor - sideSupport0) * Rangef::ZeroOneRange().limit(ratioBase / duration);
  sideSwing = sideSwing0 + (useSideStep * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor) - sideSwing0) * parabolicStep(ratioBase, duration);

  // Forward translation
  forwardSupport = forwardSupport0 + (-useForwardStep * 0.5f - forwardSupport0) * Rangef::ZeroOneRange().limit(ratio / duration);
  forwardSwing = forwardSwing0 + (useForwardStep * 0.5f - forwardSwing0) * parabolicStep(ratio, duration); // swing-foot follow-through

  // Rotation
  turnVal = turnRL0 + (swingSign * useTurnStep * 0.5f - turnRL0) * parabolicStep(ratioBase, duration);

  // Feet height
  const Rangef range(-engine.speedParameters.fastFeetAdjustment * Constants::motionCycleTime, engine.speedParameters.fastFeetAdjustment * Constants::motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeSlow(-engine.speedParameters.slowFeetAdjustment * Constants::motionCycleTime, engine.speedParameters.slowFeetAdjustment * Constants::motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeAfterKick(-engine.speedParameters.afterKickFeetHeightAdjustment * Constants::motionCycleTime, engine.speedParameters.afterKickFeetHeightAdjustment * Constants::motionCycleTime);
  if(footHeightSupport0 < 0.f && footHeightSupport0 >= engine.speedParameters.stretchSwingHeightValue)
    footHeightSupport0 -= rangeSlow.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  else
    footHeightSupport0 -= useSlowSupportFootHeightAfterKickInterpolation ? rangeAfterKick.limit(footHeightSupport0) : range.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  footHeightSupport = footHeightSupport0;

  footHeightSwing = maxFootHeight * parabolicFootHeight(ratioBase, heightDuration); // lift swing foot
  // Stretch out swing foot, if the step takes already too long, to stabilize the robots pendulum like swinging
  if(ratioBase > duration + engine.speedParameters.stretchSwingHeightAfterThisOvertime && engine.theGroundContactState.contact)
  {
    if(footHeightSwing0 > engine.speedParameters.stretchSwingHeightValue)
      footHeightSwing0 = std::max(engine.speedParameters.stretchSwingHeightValue, footHeightSwing0 + engine.speedParameters.stretchSwingHeightSpeed * Constants::motionCycleTime);
  }
  else
    footHeightSwing0 -= range.limit(footHeightSwing0); // return swing foot offset to 0
  footHeightSwing += footHeightSwing0;
}

void WalkPhase::calcWalkKickFootOffsets(const float swingSign, const float ratio,
                                        float& forwardSwing0, float& forwardSupport0, float& forwardSwing,
                                        float& forwardSupport, float& sideSwing0, float& sideSupport0,
                                        float& sideSwing, float& sideSupport, float& footHeightSwing0,
                                        float& footHeightSupport0, float& footHeightSwing,
                                        float& footHeightSupport, Angle& turnVal, WalkKickStep& walkKickStep,
                                        Angle& turnRL0)
{
  // Get current keyframe index and duration
  int keyframeIndex = 0;
  float finishedDuration = 0.f;
  for(size_t i = 0; i < walkKickStep.keyframe.size(); i++)
  {
    if(walkKickStep.keyframe[i].reachedWaitPosition)
    {
      finishedDuration += stepDuration * walkKickStep.keyframe[i].stepRatio;
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
    if(ratio != Constants::motionCycleTime)
    {
      forwardSupport0 = forwardSupport;
      forwardSwing0 = forwardSwing;
      sideSwing0 = sideSwing;
      sideSupport0 = sideSupport;
      turnRL0 = turnVal;
    }
  }

  // Get current step targets and other parameters
  const Angle useTurnStep = walkKickStep.keyframe[keyframeIndex].stepTargetConverted.rotation;
  const float useForwardStep = walkKickStep.keyframe[keyframeIndex].stepTargetConverted.translation.x();
  const float useSideStep = walkKickStep.keyframe[keyframeIndex].stepTargetConverted.translation.y();
  const float swingSpeedUp = walkKickStep.keyframe[keyframeIndex].speedUpSwing;
  const Vector2f offsetSwingFoot = walkKickStep.keyframe[keyframeIndex].stepTargetSwingConverted.translation - walkKickStep.keyframe[keyframeIndex].stepTargetConverted.translation;
  const WalkKickStep::InterpolationType interpolationType = walkKickStep.keyframe[keyframeIndex].interpolationType;

  sideSupport = sideSupport0 + (-useSideStep * engine.kinematicParameters.sidewaysHipShiftFactor - sideSupport0) * Rangef::ZeroOneRange().limit(useRatio / useDuration);
  sideSwing = sideSwing0 + (useSideStep * (1.f - engine.kinematicParameters.sidewaysHipShiftFactor) + offsetSwingFoot.y() - sideSwing0) * swingInterpolation(useRatio * swingSpeedUp, useDuration, interpolationType);

  if(!walkKickStep.keyframe[keyframeIndex].holdXSupportTarget)
    forwardSupport = forwardSupport0 + (-useForwardStep * 0.5f - forwardSupport0) * Rangef::ZeroOneRange().limit(useRatio / useDuration);
  else
    forwardSupport = forwardSupport0;

  if(!walkKickStep.keyframe[keyframeIndex].holdXSwingTarget)
    forwardSwing = forwardSwing0 + (useForwardStep * 0.5f + offsetSwingFoot.x() - forwardSwing0) * swingInterpolation(useRatio * swingSpeedUp, useDuration, interpolationType); // swing-foot follow-through
  else
    forwardSwing = forwardSwing0;

  turnVal = turnRL0 + (swingSign * useTurnStep * 0.5f - turnRL0) * Rangef::ZeroOneRange().limit(useRatio / useDuration);

  footHeightSwing = maxFootHeight * parabolicFootHeight(ratio, stepHeightDuration); // lift swing foot
  const Rangef range(-engine.speedParameters.slowFeetAdjustment * Constants::motionCycleTime, engine.speedParameters.slowFeetAdjustment * Constants::motionCycleTime); // height can adjust to 0 this much per motion frame
  footHeightSwing0 -= range.limit(footHeightSwing0); // return swing foot offset to 0
  footHeightSwing += footHeightSwing0;
  footHeightSwing = ratio / stepHeightDuration > 0.5f ? std::max(footHeightSwing, walkKickStep.reduceSwingFootHeight) : footHeightSwing;
  footHeightSupport0 -= range.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  footHeightSupport = footHeightSupport0;
}

void WalkingEngine::calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR,
                                  const float footHL, const float footHR, const Angle turn,
                                  const Angle soleRotationYL, const Angle soleRotationXL, const Angle soleRotationYR, const Angle soleRotationXR,
                                  Pose3f& leftFoot, Pose3f& rightFoot) const
{
  leftFoot = Pose3f(0, theRobotDimensions.yHipOffset, 0)
             .translate(forwardL - kinematicParameters.torsoOffset, sideL, -(kinematicParameters.walkHipHeight - footHL))
             .rotateZ(turn)
             .rotateY(soleRotationYL)
             .rotateX(soleRotationXL);
  rightFoot = Pose3f(0, -theRobotDimensions.yHipOffset, 0)
              .translate(forwardR  - kinematicParameters.torsoOffset, sideR, -(kinematicParameters.walkHipHeight - footHR))
              .rotateZ(-turn)
              .rotateY(soleRotationYR)
              .rotateX(soleRotationXR);
}

void WalkPhase::setArms(Pose3f& leftFoot, Pose3f rightFoot, JointRequest& jointRequest, JointRequest& walkArms)
{
  walkArms.angles[Joints::lShoulderPitch] = 90_deg + (leftFoot.translation.x() + engine.kinematicParameters.torsoOffset + walkStepAdjustment.lastLeftAdjustmentX) * engine.armParameters.armShoulderPitchFactor / 1000.f;
  walkArms.angles[Joints::lShoulderRoll] = engine.armParameters.armShoulderRoll + std::abs(leftFoot.translation.y() - engine.theRobotDimensions.yHipOffset) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  walkArms.angles[Joints::lElbowYaw] = 0_deg;
  walkArms.angles[Joints::lElbowRoll] = 0_deg;
  walkArms.angles[Joints::lWristYaw] = -90_deg;
  walkArms.angles[Joints::lHand] = 0.f;
  walkArms.angles[Joints::rShoulderPitch] = 90_deg + (rightFoot.translation.x() + engine.kinematicParameters.torsoOffset + walkStepAdjustment.lastRightAdjustmentX) * engine.armParameters.armShoulderPitchFactor / 1000.f;
  walkArms.angles[Joints::rShoulderRoll] = -walkArms.angles[Joints::lShoulderRoll];
  walkArms.angles[Joints::rElbowYaw] = 0_deg;
  walkArms.angles[Joints::rElbowRoll] = 0_deg;
  walkArms.angles[Joints::rWristYaw] = 90_deg;
  walkArms.angles[Joints::rHand] = 0.f;

  // Override ignore arm joints
  if(jointRequest.angles[Joints::lShoulderPitch] != JointAngles::ignore)
  {
    MotionUtilities::copy(jointRequest, leftArm, engine.theStiffnessSettings, Joints::firstLeftArmJoint, Joints::firstRightArmJoint);
    leftArmInterpolationStart = engine.theFrameInfo.time;
    leftArmInterpolationTime = std::max(std::abs(leftArm.angles[Joints::lShoulderPitch]), std::abs(leftArm.angles[Joints::lShoulderRoll] - engine.armParameters.armShoulderRoll)) / engine.standInterpolationVelocity * 1000.f;
  }
  if(jointRequest.angles[Joints::rShoulderPitch] != JointAngles::ignore)
  {
    MotionUtilities::copy(jointRequest, rightArm, engine.theStiffnessSettings, Joints::firstRightArmJoint, Joints::firstLegJoint);
    rightArmInterpolationStart = engine.theFrameInfo.time;
    rightArmInterpolationTime = std::max(std::abs(rightArm.angles[Joints::rShoulderPitch]), std::abs(rightArm.angles[Joints::rShoulderRoll] - engine.armParameters.armShoulderRoll)) / engine.standInterpolationVelocity * 1000.f;
  }

  // Interpolate left arm
  if(jointRequest.angles[Joints::lShoulderPitch] == JointAngles::ignore)
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
  if(jointRequest.angles[Joints::rShoulderPitch] == JointAngles::ignore)
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

void WalkPhase::compensateArms(Pose3f& leftFoot, Pose3f& rightFoot, JointRequest& jointRequest, const JointRequest& walkArms)
{
  JointRequest temp = walkArms;
  for(int joint = 0; joint < Joints::firstArmJoint; ++joint) // head
    temp.angles[joint] = temp.angles[joint] != JointRequest::off && temp.angles[joint] != JointRequest::ignore
                         ? temp.angles[joint] : engine.theJointAngles.angles[joint];
  const RobotModel withWalkGeneratorArms(temp, engine.theRobotDimensions, engine.theMassCalibration); // walk arms
  for(int joint = Joints::firstArmJoint; joint < Joints::firstLegJoint; ++joint) // with arm engine request
    temp.angles[joint] = jointRequest.angles[joint] != JointRequest::off
                         ? jointRequest.angles[joint] : engine.theJointAngles.angles[joint];
  RobotModel balanced(temp, engine.theRobotDimensions, engine.theMassCalibration);

  VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f(0.f, 0.f), temp, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);
  ForwardKinematic::calculateLegChain(Legs::left, temp, engine.theRobotDimensions, balanced.limbs);
  ForwardKinematic::calculateLegChain(Legs::right, temp, engine.theRobotDimensions, balanced.limbs);
  balanced.updateCenterOfMass(engine.theMassCalibration);
  armCompensationAfterKick = std::max(0.f, armCompensationAfterKick - Constants::motionCycleTime * 1000.f / engine.kinematicParameters.baseWalkPeriod);
  armCompensationTilt = (balanced.centerOfMass.x() - withWalkGeneratorArms.centerOfMass.x()) * (1.f - armCompensationAfterKick) * engine.armParameters.comTiltFactor;

  PLOT("module:WalkingEngine:armCompensation", armCompensationTilt.toDegrees());
}
