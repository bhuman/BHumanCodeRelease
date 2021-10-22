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
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Streams/InStreams.h"
#include <cmath>

MAKE_MODULE(WalkingEngine, motionControl);

WalkingEngine::WalkingEngine()
{
  InMapFile stream("walkingEngineCommon.cfg");
  if(stream.exists())
    stream >> static_cast<WalkingEngineCommon&>(*this);

  const DummyPhase dummy(MotionPhase::playDead);
  WalkPhase phase(*this, Pose2f(), dummy);
  translationPolygon = phase.getTranslationPolygon();

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

std::vector<Vector2f> WalkPhase::getTranslationPolygon()
{
  // Get max possible requests
  Vector2f backRight, frontLeft;
  const float tFactor = 1.f;
  backRight.x() = tFactor * -engine.maxSpeedBackwards * engine.baseWalkPeriod / 1000.f;
  backRight.y() = tFactor * -2.f * engine.maxSpeed.translation.y() * engine.baseWalkPeriod / 1000.f;
  frontLeft.x() = tFactor * engine.maxSpeed.translation.x() * engine.baseWalkPeriod / 1000.f ;
  frontLeft.y() = tFactor * 2.f * engine.maxSpeed.translation.y() * engine.baseWalkPeriod / 1000.f ;

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

        calcFootOffsets(1.f, 1.f, 1.f, 1.f, 0.f, 0.f, fL, fR, 0.f, 0.f, sL, sR, fLH0, fRH0, fHL, fHR, turn, forwardStep, sideStep, 0_deg);
        Pose3f leftFoot;
        Pose3f rightFoot;
        calcFeetPoses(fL, fR, sL, sR, fHL, fHR, turn, leftFoot, rightFoot); // current request
        leftFoot.translation.x() += engine.translationPolygonSafeRange.min; // in case arms are on the back, the body is shifted
        rightFoot.translation.x() += engine.translationPolygonSafeRange.min;
        if(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions))
        {
          leftFoot.translation.x() += -engine.translationPolygonSafeRange.min + engine.translationPolygonSafeRange.max; // some room to the balancer
          rightFoot.translation.x() += -engine.translationPolygonSafeRange.min + engine.translationPolygonSafeRange.max;
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

  // TODO: This makes "slow walk areas" impossible. Not that this is a bad thing...
  if(theGlobalOptions.slowWalk)
  {
    maxSpeed.translation.x() = std::min(maxSpeed.translation.x(), slowMaxSpeed.translation.x());
    maxSpeed.translation.y() = std::min(maxSpeed.translation.y(), slowMaxSpeed.translation.y());
    maxSpeed.rotation = std::min(maxSpeed.rotation, slowMaxSpeed.rotation);
    maxSpeedBackwards = std::min(maxSpeedBackwards, slowMaxSpeedBackwards);
    maxAcceleration.x() = std::min(maxAcceleration.x(), slowMaxAcceleration.x());
    maxAcceleration.y() = std::min(maxAcceleration.y(), slowMaxAcceleration.y());
  }

  lastFilteredGyroY = filteredGyroY;
  filteredGyroX = gyroLowPassRatio * filteredGyroX + (1.f - gyroLowPassRatio) * theInertialData.gyro.x();
  filteredGyroY = gyroLowPassRatio * filteredGyroY + (1.f - gyroLowPassRatio) * theInertialData.gyro.y();

  // Decide if the gyroBalanceFactors shall be taken from the WalkLearner.
  if(theWalkModifier.numOfGyroPeaks > 0)
  {
    if(theWalkLearner.newGyroBackwardBalance > 0)
    {
      gyroBackwardBalanceFactor = theWalkLearner.newGyroBackwardBalance;
    }
    if(theWalkLearner.newGyroForwardBalance > 0)
    {
      gyroForwardBalanceFactor = theWalkLearner.newGyroForwardBalance;
    }
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
      const bool predictedSwitch = std::abs(lastWalkPhase.turnStep) < turnThresholdFootSupportPrediction && theFootSupport.predictedSwitched && useFootSupportSwitchPrediction;
      const bool nonPredictedSwitch = (lastWalkPhase.t > supportSwitchPhaseRange.min * lastWalkPhase.stepDuration && (theFootSupport.switched || lastWalkPhase.robotIsNotMoving >= emergencyNotMovingCounter)) ||
                                      lastWalkPhase.t > supportSwitchPhaseRange.max * lastWalkPhase.stepDuration;
      if(predictedSwitch && !nonPredictedSwitch)
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
    const float innerTurn = 2.f * insideTurnRatio * maxSpeed.rotation * std::abs(walkSpeedRatio.rotation) * baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - insideTurnRatio) * maxSpeed.rotation * std::abs(walkSpeedRatio.rotation) * baseWalkPeriod / 1000.f;
    return Rangea(isLeftPhase ? -innerTurn : -outerTurn, isLeftPhase ? outerTurn : innerTurn);
  };

  walkGenerator.getTranslationPolygon = [this](const bool isLeftPhase, float rotation, const MotionPhase& lastPhase, const Pose2f& walkSpeedRatio, std::vector<Vector2f>& translationPolygon, const bool fastWalk)
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
        useWalkSpeedRatio.translation.x() *= walkSpeedReductionFactor;
      if(lastWalkPhase.noFastTranslationPolygonSteps > 0)
        useFastWalk = false;
      if(lastWalkPhase.doBalanceSteps > 0)
      {
        Pose3f currentComInFloor = (theTorsoMatrix * (!isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight)).inverse() * (theTorsoMatrix * theRobotModel.centerOfMass);
        Pose3f ref = (lastPhase.type == MotionPhase::walk ? static_cast<const WalkPhase&>(lastPhase).lastComInFoot : currentComInFloor);
        const Pose3f supportFoot = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
        const Pose3f oldSupportFoot = theTorsoMatrix * (static_cast<const WalkPhase&>(lastPhase).isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);

        ref = supportFoot.inverse() * oldSupportFoot * ref;

        Vector2f diff = currentComInFloor.translation.head<2>() - ref.translation.head<2>();

        Vector2f predictedCom = Pose3f(theTorsoMatrix * theRobotModel.centerOfMass).translation.head<2>() + diff * 0.5f;

        overrideForward = true;
        forwardBalance = Vector2f(predictedCom.x(), predictedCom.x());
      }
      armsAreBack = theFrameInfo.getTimeSince(lastWalkPhase.leftArmInterpolationStart) < Constants::motionCycleTime * 2.f * 1000.f || theFrameInfo.getTimeSince(lastWalkPhase.rightArmInterpolationStart) < Constants::motionCycleTime * 2.f * 1000.f;
    }

    Vector2f frontLeft, backRight;
    const Pose2f lastStep = lastPhase.type == MotionPhase::walk ? static_cast<const WalkPhase&>(lastPhase).step : Pose2f();
    const Vector2f maxStepSizeChange = maxAcceleration * baseWalkPeriod / 1000.f;
    const Vector2f maxStepSizeChangeToZero = maxDeceleration * baseWalkPeriod / 1000.f;
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

    // Limit to maximum speed (which is influenced by the rotation).
    const Angle useReduceFactor = armsAreBack ? 10_deg : reduceTranslationFromRotation.x();
    const float tFactorX = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceFactor) / (noTranslationFromRotation.x() - useReduceFactor))));
    const float reduceTranstionY = !useFastWalk ? noTranslationFromRotation.y() : ((isLeftPhase && rotation > 0_deg) || (!isLeftPhase && rotation < 0_deg) ? noTranslationYFromRotationFastOuter : noTranslationYFromRotationFastInner);
    const float useReduceTranslationFromRotation = !useFastWalk ? reduceTranslationFromRotation.y() : reduceTranslationYFromRotationFast;
    const float tFactorY = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceTranslationFromRotation) / (reduceTranstionY - useReduceTranslationFromRotation))));
    backRight.x() = std::max(backRight.x(), tFactorX * -maxSpeedBackwards * baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()));
    backRight.y() = std::max(backRight.y(), tFactorY * -2.f * maxSpeed.translation.y() * baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.y()));
    frontLeft.x() = std::min(frontLeft.x(), tFactorX * maxSpeed.translation.x() * baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()));
    frontLeft.y() = std::min(frontLeft.y(), tFactorY * 2.f * maxSpeed.translation.y() * baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.y()));

    if(overrideForward)
    {
      frontLeft.x() = forwardBalance.y();
      backRight.x() = forwardBalance.x();
    }

    if(useFastWalk)
    {
      frontLeft.x() = std::max(minXForwardTranslationFast * useWalkSpeedRatio.translation.x(), frontLeft.x());
      backRight.x() = std::min(minXBackwardTranslationFast * useWalkSpeedRatio.translation.x(), backRight.x());
    }

    // Step size in x translation has a min size
    const float maxMinStepX = std::min(minXTranslationStep, (useWalkSpeedRatio.translation.x() >= 0.f ? maxSpeed.translation.x() : maxSpeedBackwards) * baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()) + 0.01f);

    // (0,0) must be part of the rectangle.
    backRight.x() = std::min(backRight.x(), -maxMinStepX); // maxMinStepX was not in the fast walk. TODO test it on real robot
    frontLeft.x() = std::max(frontLeft.x(), maxMinStepX);
    backRight.y() = std::min(backRight.y(), -.01f);
    frontLeft.y() = std::max(frontLeft.y(), .01f);

    generateTranslationPolygon(translationPolygon, backRight, frontLeft);
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
      left.translate(lastRobotModel.soleLeft.translation.head<2>() + Vector2f(0.f, torsoOffset)).rotate(lastRobotModel.soleLeft.rotation.getZAngle());
      right.translate(lastRobotModel.soleRight.translation.head<2>() + Vector2f(0.f, torsoOffset)).rotate(lastRobotModel.soleRight.rotation.getZAngle());
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
      stepChange.translation.x() = std::max(std::abs(lastWalkPhase.forwardR0 - lastWalkPhase.forwardR), std::abs(lastWalkPhase.forwardL0 - lastWalkPhase.forwardL));
      stepChange.translation.y() = std::max(std::abs(lastWalkPhase.sideR0 - lastWalkPhase.sideR), std::abs(lastWalkPhase.sideL0 - lastWalkPhase.sideL));
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

  // adjust y forward
  Geometry::Line lineForwardAdjusted(translationPolygonTemp[1], (translationPolygonTemp[1] - translationPolygonTemp[2]).normalized());
  Vector2f leftY;
  if(Geometry::getIntersectionOfLineAndConvexPolygon(this->translationPolygon, lineForwardAdjusted, leftY))
  {
    translationPolygonTemp[1].y() = std::min(leftY.y(), translationPolygonTemp[0].y());
    translationPolygonTemp[2].y() = std::max(-leftY.y(), translationPolygonTemp[3].y());
  }

  // adjust y backward
  Geometry::Line lineBackAdjusted(translationPolygonTemp[5], (translationPolygonTemp[6] - translationPolygonTemp[5]).normalized());
  if(Geometry::getIntersectionOfLineAndConvexPolygon(this->translationPolygon, lineBackAdjusted, leftY))
  {
    translationPolygonTemp[5].y() = std::max(-leftY.y(), translationPolygonTemp[4].y());
    translationPolygonTemp[6].y() = std::min(leftY.y(), translationPolygonTemp[7].y());
  }

  polygon.clear();

  for(size_t i = 0; i < translationPolygonTemp.size(); ++i)
  {
    const Vector2f& p1 = translationPolygonTemp[i];
    const Vector2f& p2 = translationPolygonTemp[(i + 1) % translationPolygonTemp.size()];
    if(p1 != p2)
      polygon.emplace_back(p1);
  }
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
  walkingEngineOutput.maxSpeed = maxSpeed;
  walkingEngineOutput.maxSpeedBackwards = maxSpeedBackwards;
  walkingEngineOutput.walkStepDuration = baseWalkPeriod / 1000.f;
}

float WalkingEngine::getSideSpeed(const float sideTarget) const
{
  if(sidewaysWalkHeightPeriodIncreaseFactor > 0.f)
    return (std::sqrt(2000.f * sidewaysWalkHeightPeriodIncreaseFactor * std::abs(sideTarget) + baseWalkPeriod * baseWalkPeriod) - baseWalkPeriod) / (2.f * sidewaysWalkHeightPeriodIncreaseFactor);
  return 500.f * std::abs(sideTarget) / baseWalkPeriod;
}

float WalkingEngine::stepDurationSpeedTarget(const float sideSpeed) const
{
  return (baseWalkPeriod + std::abs(sideSpeed) * sidewaysWalkHeightPeriodIncreaseFactor) / 1000.f;
}

WalkPhase::WalkPhase(const WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase,
                     const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback,
                     const WalkKickStep& walkKickStep) :
  MotionPhase(MotionPhase::walk),
  engine(engine),
  walkKickStep(walkKickStep),
  createNextPhaseCallback(createNextPhaseCallback)
{
  Pose2f useStepTarget = SystemCall::getMode() == SystemCall::logFileReplay ? engine.theWalkStepData.stepTarget : stepTarget;
  if(lastPhase.type == MotionPhase::kick && useStepTarget == Pose2f())
    useStepTarget.translation.x() = 0.01f;
  const bool standRequested = (useStepTarget.translation.x() == 0.f && useStepTarget.translation.y() == 0.f && useStepTarget.rotation == 0_deg) || !engine.theGroundContactState.contact;
  timeWhenStandBegan = engine.theFrameInfo.time;
  const bool afterKickOrGetUp = (lastPhase.type == MotionPhase::kick || lastPhase.type == MotionPhase::getUp) && !standRequested;
  engine.theEnergySaving.shutDown();
  leftArmInterpolationStart = engine.theFrameInfo.time;
  rightArmInterpolationStart = engine.theFrameInfo.time;
  leftArm = engine.theJointRequest;
  rightArm = engine.theJointRequest;
  Pose2f lastStepTarget;
  lastLeftSole = Pose2f(engine.theRobotModel.soleLeft.rotation.getZAngle(), engine.theRobotModel.soleLeft.translation.head<2>());
  lastRightSole = Pose2f(engine.theRobotModel.soleRight.rotation.getZAngle(), engine.theRobotModel.soleRight.translation.head<2>());
  currentLeftSole = lastLeftSole;
  currentRightSole = lastRightSole;
  standInterpolationDuration = 1000.f; // default value

  leftArmInterpolationTime = rightArmInterpolationTime = engine.armInterpolationTime;
  calculateBallPosition();

  // Init variables with the last walk phase
  auto initVariablesLastPhase = [this](const WalkPhase& lastWalkPhase)
  {
    armCompensationAfterKick = lastWalkPhase.armCompensationAfterKick;
    annotationTimestamp = lastWalkPhase.annotationTimestamp;
    leftArmInterpolationStart = lastWalkPhase.leftArmInterpolationStart;
    rightArmInterpolationStart = lastWalkPhase.rightArmInterpolationStart;
    leftArmInterpolationTime = lastWalkPhase.leftArmInterpolationTime;
    rightArmInterpolationTime = lastWalkPhase.rightArmInterpolationTime;
    leftArm = lastWalkPhase.leftArm;
    rightArm = lastWalkPhase.rightArm;
    forwardL0 = lastWalkPhase.forwardL;
    forwardR0 = lastWalkPhase.forwardR;
    sideL0 = lastWalkPhase.lastPrevSideL;
    sideR0 = lastWalkPhase.lastPrevSideR;
    turnRL0 = lastWalkPhase.turnRL;
    footHL0 = lastWalkPhase.footHL;
    footHR0 = lastWalkPhase.footHR;
    forwardL0 += lastWalkPhase.walkStepAdjustment.lastLeftAdjustmentX;
    forwardR0 += lastWalkPhase.walkStepAdjustment.lastRightAdjustmentX;
    soleRotationYL = lastWalkPhase.soleRotationYL;
    soleRotationXL = lastWalkPhase.soleRotationXL;
    soleRotationYR = lastWalkPhase.soleRotationYR;
    soleRotationXR = lastWalkPhase.soleRotationXR;
    afterWalkKickPhase = lastWalkPhase.walkKickStep.currentKick != WalkKicks::none && this->walkKickStep.currentKick == WalkKicks::none;
    lastLeftSole = lastWalkPhase.lastLeftSole;
    lastRightSole = lastWalkPhase.lastRightSole;
    noFastTranslationPolygonSteps = std::max(0, lastWalkPhase.noFastTranslationPolygonSteps - 1);
    armCompensationTilt = lastWalkPhase.armCompensationTilt;
    useSlowSupportFootHeightInterpolation = lastWalkPhase.walkKickStep.useSlowFootHeightInterpolation;
    lastWalkPhaseKneeHipBalance = weightShiftStatus == weightDidShift ? lastWalkPhase.currentWalkPhaseKneeHipBalance : 0_deg;
    lastWalkPhaseKneeHipBalance -= Rangea(-this->engine.soleRotationOffsetSpeed * Constants::motionCycleTime, this->engine.soleRotationOffsetSpeed * Constants::motionCycleTime).limit(lastWalkPhaseKneeHipBalance);
    gyroStateTimestamp = lastWalkPhase.gyroStateTimestamp;

    walkStepAdjustment.init(lastWalkPhase.walkStepAdjustment, hipGyroBalance, this->engine.hipBalanceSteps, true, this->engine.theFrameInfo, this->engine.unstableWalkThreshold);
  };

  // init start positions for the feet variables based on the last joint request
  auto overrideStartPositions = [this](const bool leftReplace, const bool rightReplace)
  {
    JointRequest lastRequest = this->engine.theJointRequest;
    lastRequest.angles[Joints::lHipPitch] -= armCompensationTilt;
    lastRequest.angles[Joints::rHipPitch] -= armCompensationTilt;
    FOREACH_ENUM(Joints::Joint, joint)
    {
      lastRequest.angles[joint] = lastRequest.angles[joint] == JointAngles::off || lastRequest.angles[joint] == JointAngles::ignore
                                  ? this->engine.theJointAngles.angles[joint]
                                  : lastRequest.angles[joint];
    }
    const RobotModel lastRobotModel(lastRequest, this->engine.theRobotDimensions, this->engine.theMassCalibration);
    if(leftReplace)
    {
      sideL0 = (lastRobotModel.soleLeft.translation.y() - this->engine.theRobotDimensions.yHipOffset);
      forwardL0 = (lastRobotModel.soleLeft.translation.x() + this->engine.torsoOffset);
      footHL0 = (lastRobotModel.soleLeft.translation.z() + this->engine.walkHipHeight);
      soleRotationYL = lastRobotModel.soleLeft.rotation.getYAngle();
      soleRotationXL = lastRobotModel.soleLeft.rotation.getXAngle();
    }
    if(rightReplace)
    {
      sideR0 = (lastRobotModel.soleRight.translation.y() + this->engine.theRobotDimensions.yHipOffset);
      forwardR0 = (lastRobotModel.soleRight.translation.x() + this->engine.torsoOffset);
      footHR0 = (lastRobotModel.soleRight.translation.z() + this->engine.walkHipHeight);
      soleRotationYR = lastRobotModel.soleRight.rotation.getYAngle();
      soleRotationXR = lastRobotModel.soleRight.rotation.getXAngle();
    }
    turnRL0 = (lastRobotModel.soleLeft.rotation.getZAngle() - lastRobotModel.soleRight.rotation.getZAngle()) / 2.f;
  };

  // Previous phase was a walk phase
  if(lastPhase.type == MotionPhase::walk)
  {
    if(afterKickOrGetUp)
    {
      this->createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback();
      this->walkKickStep = WalkKickStep();
    }
    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    const bool predictedSwitch = std::abs(lastWalkPhase.turnStep) < engine.turnThresholdFootSupportPrediction && engine.theFootSupport.predictedSwitched && engine.useFootSupportSwitchPrediction;
    const bool nonPredictedSwitch = (lastWalkPhase.t > engine.supportSwitchPhaseRange.min * lastWalkPhase.stepDuration && (engine.theFootSupport.switched || lastWalkPhase.robotIsNotMoving >= engine.emergencyNotMovingCounter)) ||
                                    lastWalkPhase.t > engine.supportSwitchPhaseRange.max * lastWalkPhase.stepDuration;
    const bool predictBasedSwitch = predictedSwitch && !nonPredictedSwitch;
    ASSERT(lastWalkPhase.walkState != standing);

    weightShiftStatus = (!predictBasedSwitch && lastWalkPhase.isLeftPhase != (engine.theFootSupport.support < 0.f)) ||
                        (predictBasedSwitch && lastWalkPhase.isLeftPhase != (engine.theFootSupport.support > 0.f)) ? weightDidShift : weightDidNotShift;
    isLeftPhase = predictBasedSwitch ? engine.theFootSupport.support > 0.f : engine.theFootSupport.support < 0.f;
    if(lastWalkPhase.earlySupportSwitchAllowed && lastWalkPhase.t < engine.supportSwitchPhaseRange.min * lastWalkPhase.stepDuration)
      useStepTarget.translation.x() = -30.f;

    lastStepTarget = lastWalkPhase.step;
    doBalanceSteps = std::max(0, lastWalkPhase.doBalanceSteps - 1);
    walkState = standRequested ? stopping : walking;

    // if robot is stopping, we need 1-2 walk phases for it
    if(walkState == stopping && lastWalkPhase.walkState == stopping)
    {
      if(isStandingPossible(lastWalkPhase.forwardL, lastWalkPhase.forwardR, lastWalkPhase.lastPrevSideL, lastWalkPhase.lastPrevSideR, lastWalkPhase.turnRL, lastWalkPhase.walkStepAdjustment.lastLeftAdjustmentX, lastWalkPhase.walkStepAdjustment.lastRightAdjustmentX))
      {
        walkState = standing;
        stepDuration = engine.stepDurationSpeedTarget(engine.getSideSpeed(0.f));
        stepHeightDuration = engine.stepDurationSpeedTarget(engine.getSideSpeed(0.f));
      }
    }

    initVariablesLastPhase(lastWalkPhase);

    // Detection does not work when the forward long was executed
    if(lastWalkPhase.walkKickStep.currentKick == WalkKicks::forwardLong)
      for(unsigned i = 0; i < engine.feetHeightDifferenceNumberOfSamples; i++)
        supportSwingHeightDifference.push_back(0.f);

    // Walk backwards to walk away from the obstacle
    if(lastWalkPhase.earlySupportSwitchAllowed && lastWalkPhase.t / lastWalkPhase.stepDuration < engine.maxSupportSwitchPhaseRangeAfterSteppingOnOpponentFeet)
    {
      useStepTarget.translation.x() = engine.stepSizeXAfterWalkingOnOpponentFeet; // walk backwards
      useStepTarget.translation.y() = 0.f; // pull both feet together
    }

    if(lastWalkPhase.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::measured || lastWalkPhase.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::measured)
      adjustRequestByMeasuredPosition(lastWalkPhase.isLeftPhase, lastWalkPhase.t, lastWalkPhase.stepDuration, lastWalkPhase.walkKickStep);
    if(walkKickStep.numOfBalanceSteps > 0)
      doBalanceSteps = walkKickStep.numOfBalanceSteps;

    hipGyroBalance = lastWalkPhase.hipGyroBalance;

    // Last WalkPhase was an InWalkKick. Calculate the requests of the last support foot. This is done, because of the HipPitch balancing and to get a smooth interpolation
    if(lastWalkPhase.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::request || lastWalkPhase.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::request)
      overrideStartPositions((!lastWalkPhase.isLeftPhase && isLeftPhase && lastWalkPhase.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::request) ||
                             (lastWalkPhase.isLeftPhase && !isLeftPhase && lastWalkPhase.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::request),
                             (!lastWalkPhase.isLeftPhase && isLeftPhase && lastWalkPhase.walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::request) ||
                             (lastWalkPhase.isLeftPhase && !isLeftPhase && lastWalkPhase.walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::request));

    if(walkState != standing)
    {
      lastSideStep = lastWalkPhase.sideStep;
      prevSideL = sideL0;
      prevSideR = sideR0;
      prevTurn = lastWalkPhase.prevTurn;
    }
    if(engine.theGroundContactState.contact) // should not be !standRequest, because robot could be stuck at a goal post
    {
      if(weightShiftStatus == weightDidNotShift && lastWalkPhase.weightShiftStatus == emergencyStand && lastWalkPhase.robotIsNotMoving >= engine.emergencyNotMovingCounter)
        weightShiftStatus = emergencyStep;
      else if(weightShiftStatus == weightDidNotShift)
        weightShiftStatus = emergencyStand;
    }
    engine.theWalkStepData.updateCounter(predictedSwitch && !nonPredictedSwitch);
  }

  // Previous phase was a stand phase
  else if(lastPhase.type == MotionPhase::stand)
  {
    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    initVariablesLastPhase(lastWalkPhase);

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
      sideL0 = lastWalkPhase.sideL0;
      sideR0 = lastWalkPhase.sideR0;
    }
  }

  // Previous phase was a kick or get up phase
  else if(afterKickOrGetUp)
  {
    afterKickWalkPhase = true;
    walkState = starting;
    weightShiftStatus = weightDidShift;
    isLeftPhase = engine.theFootSupport.support < 0.f;
    overrideStartPositions(true, true);

    armCompensationAfterKick = 1.f;
    const RobotModel lastRobotModel(engine.theJointRequest, engine.theRobotDimensions, engine.theMassCalibration);

    // No detetction after get up or kicking
    for(unsigned i = 0; i < engine.feetHeightDifferenceNumberOfSamples; i++)
      supportSwingHeightDifference.push_back(0.f);

    forwardStep = ((!isLeftPhase ? lastRobotModel.soleRight.translation.x() : lastRobotModel.soleLeft.translation.x()) + engine.torsoOffset) * 2.f;
    sideStep = -(isLeftPhase ? lastRobotModel.soleRight.translation.y() + 50.f : lastRobotModel.soleLeft.translation.y() - 50.f);

    // Limit max step size
    const Rangef forwardStepRange(-engine.maxSpeedBackwards * engine.baseWalkPeriod / 1000.f, engine.maxSpeed.translation.x()* engine.baseWalkPeriod / 1000.f);
    forwardStepRange.clamp(forwardStep);

    turnStep = 0_deg;
    step = Pose2f();
  }

  // Previous phase was something else (e.g. playDead)
  else
  {
    leftArmInterpolationStart = 0;
    rightArmInterpolationStart = 0;
    Pose3f leftFoot;
    Pose3f rightFoot;
    calcFeetPoses(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, leftFoot, rightFoot); // current request
    JointRequest request;
    request.angles.fill(JointAngles::ignore);
    JointRequest arms;
    VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), request, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);
    setArms(leftFoot, rightFoot, request, arms);
    standFactor = -1.f;
    walkState = standing;
    weightShiftStatus = weightDidShift;
    startJointAngles = engine.theJointRequest;
    Angle jointDif = 0_deg;
    FOREACH_ENUM(Joints::Joint, joint)
    {
      if(joint < Joints::firstArmJoint)
        continue;
      if(startJointAngles.angles[joint] == JointAngles::off || startJointAngles.angles[joint] == JointAngles::ignore)
        startJointAngles.angles[joint] = engine.theJointAngles.angles[joint];
      const Angle dif = startJointAngles.angles[joint] - request.angles[joint];
      const float factor = joint >= Joints::firstArmJoint && joint < Joints::firstLeftLegJoint ? 2.f : 1.f; // arms are factored more than all other joints
      jointDif = std::max(jointDif, Angle(std::abs(dif * factor)));
    }
    standInterpolationDuration = jointDif / engine.standInterpolationVelocity * 1000.f;
    armCompensationAfterKick = 1.f;
    leftArmInterpolationStart = engine.theFrameInfo.time;
    rightArmInterpolationStart = engine.theFrameInfo.time;
  }

  // Previous was walking (or stopping). Calculate step size. Rotational requests must be compensated
  if(walkState != standing)
  {
    const float sideSpeed = engine.getSideSpeed(std::max(std::abs(useStepTarget.translation.y()), std::abs(lastStepTarget.translation.y())));
    stepDuration = engine.stepDurationSpeedTarget(0.f);
    stepHeightDuration = engine.stepDurationSpeedTarget(sideSpeed);

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
      noFastTranslationPolygonSteps = engine.noFastTranslationPolygonStepsNumber;
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
    maxFootHeight = engine.baseFootLift * (walkState == starting ? engine.reduceSwingHeightStartingFactor * (1.f - std::abs(sideSpeed) / engine.maxSpeed.translation.y()) + std::abs(sideSpeed) / engine.maxSpeed.translation.y() : 1.f) * walkKickStep.increaseSwingHeightFactor;
  }
  else
    type = MotionPhase::stand;

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
    sideStep = isLeftPhase ? engine.emergencyStepSize : -engine.emergencyStepSize;
    turnStep = 0_deg;
    maxFootHeight *= engine.emergencyStepHeightFactor;
    step = Pose2f();
  }

  // Prevent false time stamps when replaying logs
  if(engine.theFrameInfo.time < leftArmInterpolationStart)
    leftArmInterpolationStart = engine.theFrameInfo.time;
  if(engine.theFrameInfo.time < rightArmInterpolationStart)
    rightArmInterpolationStart = engine.theFrameInfo.time;

  // Decide if the gyroBalanceFactors shall be taken from the WalkLearner.
  if(engine.theWalkModifier.numOfGyroPeaks > 0)
    engine.theWalkLearner.setBaseWalkParams(engine.gyroForwardBalanceFactor, engine.gyroBackwardBalanceFactor, forwardStep);

  // Set walk values
  engine.theWalkStepData.updateWalkValues(step, stepDuration);

  lastComInFoot = (engine.theTorsoMatrix * (!isLeftPhase ? engine.theRobotModel.soleLeft : engine.theRobotModel.soleRight)).inverse() * (engine.theTorsoMatrix * engine.theRobotModel.centerOfMass);
  slowDownHipKneeBalanceFactor = Rangef::ZeroOneRange().limit((step.translation.x() - lastStepTarget.translation.x()) / (-engine.maxSpeed.translation.x() * (engine.baseWalkPeriod / 1000.f)));
}

void WalkPhase::update()
{
  t += Constants::motionCycleTime;
  prevTurn = turnRL;
  lastPrevSideL = prevSideL;
  lastPrevSideR = prevSideR;
  prevSideL = sideL;
  prevSideR = sideR;
  lastLeftSole = currentLeftSole;
  lastRightSole = currentRightSole;
  lastWalkPhaseKneeHipBalance -= Rangea(-engine.soleRotationOffsetSpeed * Constants::motionCycleTime, engine.soleRotationOffsetSpeed * Constants::motionCycleTime).limit(lastWalkPhaseKneeHipBalance);
  currentLeftSole = Pose2f(engine.theRobotModel.soleLeft.rotation.getZAngle(), engine.theRobotModel.soleLeft.translation.head<2>());
  currentRightSole = Pose2f(engine.theRobotModel.soleRight.rotation.getZAngle(), engine.theRobotModel.soleRight.translation.head<2>());
}

bool WalkPhase::isDone(const MotionRequest& motionRequest) const
{
  const bool predictedSwitch = std::abs(turnStep) < engine.turnThresholdFootSupportPrediction && engine.theFootSupport.predictedSwitched && engine.useFootSupportSwitchPrediction;
  return walkState == standing ? (motionRequest.motion != MotionRequest::stand && (!motionRequest.isWalking() || (engine.theGroundContactState.contact && standFactor == 0.f))) // switch from stand to walk
         : ((t > engine.supportSwitchPhaseRange.min * stepDuration && (engine.theFootSupport.switched || robotIsNotMoving >= engine.emergencyNotMovingCounter || predictedSwitch)) ||
            t > engine.supportSwitchPhaseRange.max * stepDuration ||
            (earlySupportSwitchAllowed && isLeftPhase != engine.theFootSupport.support < 0.f));
}

void WalkPhase::detectWalkingOnOpponentFeet()
{
  if(walkState == standing || !engine.theFootSoleRotationCalibration.isCalibrated ||
     supportSwingHeightDifference.size() >= engine.feetHeightDifferenceNumberOfSamples || !engine.useSteppingOnOpponentFootBehavior)
    return;

  // Fail-Safe, only if we know there is an obstacle "near", otherwise fill the list to skip checks in the current walking step
  if(supportSwingHeightDifference.empty())
  {
    bool obstacleNear = false;
    for(Obstacle ob : engine.theObstacleModel.obstacles)
    {
      obstacleNear |= ob.center.squaredNorm() < sqr(engine.maxObstacleDistance);
      if(obstacleNear)
        break;
    }
    if(!obstacleNear)
    {
      for(unsigned i = 0; i < engine.feetHeightDifferenceNumberOfSamples; i++)
        supportSwingHeightDifference.push_back(0.f);
      return;
    }
  }

  // Get soles in torso
  const Pose3f inTorsoLeft = engine.theTorsoMatrix * engine.theRobotModel.soleLeft;
  const Pose3f inTorsoRight = engine.theTorsoMatrix * engine.theRobotModel.soleRight;

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
  if(supportSwingHeightDifference.size() == engine.feetHeightDifferenceNumberOfSamples && // we have all samples
     supportSwingHeightDifference[0] < engine.minFeetHeightDifference && // first sample threshold check
     supportSwingHeightDifference[engine.feetHeightDifferenceNumberOfSamples - 1] < engine.minFeetHeightDifference && // last sample threshold check
     supportSwingHeightDifference[0] - supportSwingHeightDifference[engine.feetHeightDifferenceNumberOfSamples - 1] > engine.maxFeetHeightDifferenceVelocity * (1.f + std::max(0.f, (engine.minFeetHeightDifference - supportSwingHeightDifference[0]) / engine.maxFeetHeightDifferenceScaling)) &&
     walkStepAdjustment.previousHighestAdjustmentX > engine.maxLastBackwardStepAdjustment) // if the robot is already tilting backward, then we can be sure what is actually happening
  {
    stepHeightDuration = 0.f;
    earlySupportSwitchAllowed = true;
    ANNOTATION("WalkingEngine", "Detected walking step on opponent feet");
    // TODO in theory we also need to make sure the current swing foot gets ground contact as fast as possible.
    // Often the detection is not enough because the next support switch happens after 50% of the step duration
  }
  PLOT("module:WalkingEngine:Data:feetHeightDifference", supportSwingHeightDifference.back());
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
        calcFootOffsets(1.f, t, stepDuration, stepHeightDuration, forwardL0, forwardR0, forwardL, forwardR, sideL0, sideR0, sideL, sideR, footHL0, footHR0, footHL, footHR, turnRL, forwardStep, sideStep, turnStep);
      else
        calcFootOffsets(-1.f, t, stepDuration, stepHeightDuration, forwardR0, forwardL0, forwardR, forwardL, sideR0, sideL0, sideR, sideL, footHR0, footHL0, footHR, footHL, turnRL, forwardStep, sideStep, turnStep);
    }
    // InWalkKicks
    else
    {
      if(isLeftPhase)
        calcWalkKickFootOffsets(1.f, t, forwardL0, forwardR0, forwardL, forwardR, sideL0, sideR0, sideL, sideR, footHL0, footHR0, footHL, footHR, turnRL, walkKickStep, turnRL0);
      else
        calcWalkKickFootOffsets(-1.f, t, forwardR0, forwardL0, forwardR, forwardL, sideR0, sideL0, sideR, sideL, footHR0, footHL0, footHR, footHL, turnRL, walkKickStep, turnRL0);
    }
  }
  else
  {
    if(standFactor != 1.f)
      timeWhenStandHighBegan = engine.theFrameInfo.time;

    wasStandingStillOnce |= robotIsNotMoving >= engine.standHighNotMovingCounter;
    const bool highStandRequested = motionRequest.motion == MotionRequest::stand && motionRequest.standHigh && wasStandingStillOnce;
    if(standFactor < 0.f)
      standFactor = std::min(standFactor + 1000.f * Constants::motionCycleTime / standInterpolationDuration, 0.f);
    else if(highStandRequested)
      standFactor = std::min(standFactor + 1000.f * Constants::motionCycleTime / engine.standHighInterpolationDuration, 1.f);
    else
      standFactor = std::max(standFactor - 1000.f * Constants::motionCycleTime / engine.standHighInterpolationDuration, 0.f);

    const float standFactor01 = std::max(standFactor, 0.f);
    forwardL0 = forwardR0 = 0.5f * (1.f - std::cos(standFactor01 * pi)) * engine.torsoOffset;
    footHL0 = footHR0 = -std::sin(standFactor01 * pi_2) *
                        (engine.theRobotDimensions.upperLegLength + engine.theRobotDimensions.lowerLegLength + engine.theRobotDimensions.footHeight -
                         engine.walkHipHeight - 0.0001f);
    turnRL0 = sideL0 = sideR0 = 0.f;

    forwardL = forwardL0;
    forwardR = forwardR0;
    sideL = sideL0;
    sideR = sideR0;
    footHL = footHL0;
    footHR = footHR0;
    turnRL = turnRL0;
  }

  Pose3f leftFoot;
  Pose3f rightFoot;
  calcFeetPoses(forwardL, forwardR, sideL, sideR, footHL, footHR, turnRL, leftFoot, rightFoot); // current request

  VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), jointRequest, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);

  const bool applyEnergySavingStand = walkState == standing && standFactor == 0.f && engine.theFrameInfo.getTimeSince(timeWhenStandBegan) > engine.standStiffnessDelay;
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
    balanceFeetPoses(leftFoot, rightFoot, jointRequest);
  }

  // Add gyro balancing
  addGyroBalance(jointRequest);

  // Reset feet rotation offsets back to 0
  const Rangea limitAnkleOffset(-engine.soleRotationOffsetSpeed * Constants::motionCycleTime, engine.soleRotationOffsetSpeed * Constants::motionCycleTime);
  Angle soleYLOld = soleRotationYL;
  Angle soleYROld = soleRotationYR;
  soleRotationYL -= limitAnkleOffset.limit(soleRotationYL);
  soleRotationXL -= limitAnkleOffset.limit(soleRotationXL);
  soleRotationYR -= limitAnkleOffset.limit(soleRotationYR);
  soleRotationXR -= limitAnkleOffset.limit(soleRotationXR);

  // Add sole rotation compensation
  if(walkState == walking)
  {
    RobotModel balanced(jointRequest, engine.theRobotDimensions, engine.theMassCalibration);
    walkStepAdjustment.modifySwingFootRotation(isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft,
                                               isLeftPhase ? balanced.soleRight : balanced.soleLeft,
                                               isLeftPhase ? soleRotationYL : soleRotationYR,
                                               isLeftPhase ? soleYLOld : soleYROld,
                                               engine.theInertialData.angle.y(), t / stepDuration,
                                               engine.soleRotationOffsetSpeed, engine.soleCompensationMinTorsoRotation,
                                               engine.soleCompensationMinSupportSoleRotation, engine.theFootSoleRotationCalibration.isCalibrated, isLeftPhase,
                                               engine.soleBackwardsCompensationTorsoFactor, engine.soleForwardCompensationReturnZeroRation,
                                               engine.soleBackwardsCompensationReturnZeroRatio, engine.soleBackwardsCompensationFeetXDifference,
                                               engine.soleBackwardsCompensationFeetShift, engine.soleCompensationReduction, engine.soleCompensationIncreasement);
  }

  // Apply stand heat
  if(!(applyEnergySavingHighStand || applyEnergySavingStand) && engine.theEnergySaving.state == EnergySaving::EnergyState::working)
    engine.theEnergySaving.reset();
  if(applyEnergySavingHighStand || applyEnergySavingStand || engine.theEnergySaving.state != EnergySaving::waiting)
    engine.theEnergySaving.applyHeatAdjustment(jointRequest, true, true, standFactor == 1.f, engine.theCalibrationRequest.preciseJointPositions);

  applyWalkKickLongKickOffset(jointRequest, t);

  odometryOffset = getOdometryOffset();

  motionInfo.isMotionStable = true;
  motionInfo.isWalkPhaseInWalkKick = walkKickStep.currentKick != WalkKicks::none;
  motionInfo.speed = Pose2f(turnStep / stepDuration, forwardStep / stepDuration, sideStep / stepDuration); // TODO

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

std::unique_ptr<MotionPhase> WalkPhase::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  // There must be another walk phase if the robot should transition into something other than walking and the feet are not next to each other.
  if(defaultNextPhase.type != MotionPhase::walk && walkState != stopping && walkState != standing && isStandingPossible(forwardL, forwardR, lastPrevSideL, lastPrevSideR, turnRL, walkStepAdjustment.lastLeftAdjustmentX, walkStepAdjustment.lastRightAdjustmentX))
    return std::make_unique<WalkPhase>(engine, Pose2f(), *this);
  if(std::unique_ptr<MotionPhase> nextPhase; createNextPhaseCallback && (nextPhase = createNextPhaseCallback(*this)))
    return nextPhase;
  return std::unique_ptr<MotionPhase>();
}

Pose2f WalkPhase::getOdometryOffset() const
{
  const bool useIsLeftPhase = t > Constants::motionCycleTime ? isLeftPhase : !isLeftPhase;
  if(walkState == standing)
    return Pose2f(0_deg, 0.f, 0.f);
  const float yHipOffset = engine.theRobotDimensions.yHipOffset;
  const Vector2f hipOffset(0.f, useIsLeftPhase ? engine.theRobotDimensions.yHipOffset : -engine.theRobotDimensions.yHipOffset);
  const float sign = useIsLeftPhase ? 1.f : -1.f;

  const Vector2f currentOriginal((useIsLeftPhase ? (currentLeftSole.translation.x() - currentRightSole.translation.x()) : (currentRightSole.translation.x() - currentLeftSole.translation.x())),
                                 (useIsLeftPhase ? (currentLeftSole.translation.y() - currentRightSole.translation.y() - 2.f * yHipOffset) : (currentRightSole.translation.y() - currentLeftSole.translation.y() + 2.f * yHipOffset)));
  const Vector2f oldOriginal((useIsLeftPhase ? (lastLeftSole.translation.x() - lastRightSole.translation.x()) : (lastRightSole.translation.x() - lastLeftSole.translation.x())),
                             (useIsLeftPhase ? (lastLeftSole.translation.y() - lastRightSole.translation.y() - 2.f * yHipOffset) : (lastRightSole.translation.y() - lastLeftSole.translation.y() + 2.f * yHipOffset)));

  const Vector2f currentConverted = (currentOriginal - hipOffset.rotated(turnRL * sign) + 2.f * hipOffset).rotated(turnRL * sign) - hipOffset;
  const Vector2f oldConverted = (oldOriginal - hipOffset.rotated(prevTurn * sign) + 2.f * hipOffset).rotated(prevTurn * sign) - hipOffset;

  const Pose2f offset((turnRL - prevTurn) * sign * engine.odometryScale.rotation * 2.f,
                      (currentConverted.x() - oldConverted.x()) * engine.odometryScale.translation.x() * 0.5f,
                      (currentConverted.y() - oldConverted.y()) * engine.odometryScale.translation.y() * 0.5f);

  return offset;
}

void WalkPhase::calcFootOffsets(const float swingSign, const float ratio, const float duration, const float heightDuration,
                                const float forwardSwing0, const float forwardSupport0, float& forwardSwing,
                                float& forwardSupport, const float sideSwing0, const float sideSupport0,
                                float& sideSwing, float& sideSupport, float& footHeightSwing0,
                                float& footHeightSupport0, float& footHeightSwing,
                                float& footHeightSupport, Angle& turnVal,
                                const float useForwardStep, const float useSideStep, const Angle useTurnStep)
{
  // Side translation
  sideSupport = sideSupport0 + (-useSideStep * engine.sidewaysHipShiftFactor - sideSupport0) * Rangef::ZeroOneRange().limit(ratio / duration);
  sideSwing = sideSwing0 + (useSideStep * (1.f - engine.sidewaysHipShiftFactor) - sideSwing0) * parabolicStep(ratio, duration);

  // Forward translation
  forwardSupport = forwardSupport0 + (-useForwardStep * 0.5f - forwardSupport0) * Rangef::ZeroOneRange().limit(ratio / duration);
  forwardSwing = forwardSwing0 + (useForwardStep * 0.5f - forwardSwing0) * parabolicStep(ratio, duration); // swing-foot follow-through

  // Rotation
  turnVal = turnRL0 + (swingSign * useTurnStep * 0.5f - turnRL0) * parabolicStep(ratio, duration);

  // Feet height
  const Rangef range(-engine.fastFeetAdjustment * Constants::motionCycleTime, engine.fastFeetAdjustment * Constants::motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeSlow(-engine.slowFeetAdjustment * Constants::motionCycleTime, engine.slowFeetAdjustment * Constants::motionCycleTime); // height can adjust to 0 this much per motion frame
  const Rangef rangeAfterKick(-engine.afterKickFeetHeightAdjustment * Constants::motionCycleTime, engine.afterKickFeetHeightAdjustment * Constants::motionCycleTime);
  if(footHeightSupport0 < 0.f && footHeightSupport0 >= engine.stretchSwingHeightValue)
    footHeightSupport0 -= rangeSlow.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  else
    footHeightSupport0 -= useSlowSupportFootHeightInterpolation ? rangeAfterKick.limit(footHeightSupport0) : range.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  footHeightSupport = footHeightSupport0;

  footHeightSwing = maxFootHeight * parabolicFootHeight(ratio / heightDuration); // lift swing foot
  // Stretch out swing foot, if the step takes already too long, to stabilize the robots pendulum like swinging
  if(ratio > duration + engine.stretchSwingHeightAfterThisOvertime && engine.theGroundContactState.contact)
  {
    if(footHeightSwing0 > engine.stretchSwingHeightValue)
      footHeightSwing0 = std::max(engine.stretchSwingHeightValue, footHeightSwing0 + engine.stretchSwingHeightSpeed * Constants::motionCycleTime);
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

  sideSupport = sideSupport0 + (-useSideStep * engine.sidewaysHipShiftFactor - sideSupport0) * Rangef::ZeroOneRange().limit(useRatio / useDuration);
  sideSwing = sideSwing0 + (useSideStep * (1.f - engine.sidewaysHipShiftFactor) + offsetSwingFoot.y() - sideSwing0) * Rangef::ZeroOneRange().limit(useRatio * swingSpeedUp / useDuration);

  if(!walkKickStep.keyframe[keyframeIndex].holdXSupportTarget)
    forwardSupport = forwardSupport0 + (-useForwardStep * 0.5f - forwardSupport0) * Rangef::ZeroOneRange().limit(useRatio / useDuration);
  else
    forwardSupport = forwardSupport0;

  const float swingInterpolationRatio = walkKickStep.currentKick == WalkKicks::forwardLong && keyframeIndex > 0 ? Rangef::ZeroOneRange().limit(useRatio * swingSpeedUp / useDuration) : parabolicStep(useRatio * swingSpeedUp, useDuration);

  if(!walkKickStep.keyframe[keyframeIndex].holdXSwingTarget)
    forwardSwing = forwardSwing0 + (useForwardStep * 0.5f + offsetSwingFoot.x() - forwardSwing0) * swingInterpolationRatio; // swing-foot follow-through
  else
    forwardSwing = forwardSwing0;

  turnVal = turnRL0 + (swingSign * useTurnStep * 0.5f - turnRL0) * Rangef::ZeroOneRange().limit(useRatio / useDuration);

  footHeightSwing = maxFootHeight * parabolicFootHeight(ratio / stepHeightDuration); // lift swing foot
  const Rangef range(-engine.slowFeetAdjustment * Constants::motionCycleTime, engine.slowFeetAdjustment * Constants::motionCycleTime); // height can adjust to 0 this much per motion frame
  footHeightSwing0 -= range.limit(footHeightSwing0); // return swing foot offset to 0
  footHeightSwing += footHeightSwing0;
  footHeightSwing = ratio / stepHeightDuration > 0.5f ? std::max(footHeightSwing, walkKickStep.reduceSwingFootHeight) : footHeightSwing;
  footHeightSupport0 -= range.limit(footHeightSupport0); // return support foot to 0 if it was still lifted
  footHeightSupport = footHeightSupport0;
}

float WalkPhase::parabolicFootHeight(float f) const
{
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

float WalkPhase::parabolicStep(float time, float period) const
{
  const float timeFraction = Rangef::ZeroOneRange().limit(time / period);
  if(timeFraction < 0.5f)
    return 2.f * timeFraction * timeFraction;
  else
    return 4.f * timeFraction - 2.f * timeFraction * timeFraction - 1.f;
}

void WalkPhase::balanceFeetPoses(Pose3f& leftFoot, Pose3f& rightFoot, JointRequest& jointRequest)
{
  RobotModel lightModel = engine.theRobotModel;
  lightModel.updateCenterOfMass(engine.lightMassCalibration);

  // Predict CoM position (diff)
  Pose3f tiltInTorso(RotationMatrix(Rotation::AngleAxis::unpack(Vector3f(-engine.theInertialData.angle.x(), -engine.theInertialData.angle.y(), 0.f))));
  const Pose3f rot = walkStepAdjustment.predictRotation(tiltInTorso, lightModel, engine.theFootOffset, isLeftPhase, !afterKickWalkPhase);
  Pose3f comInTorso = (rot.inverse() * lightModel.centerOfMass);
  Pose3f sole = isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft;
  sole = rot.inverse() * sole;
  comInTorso = comInTorso * Vector3f(0.f, 0.f, sole.translation.z() - comInTorso.translation.z());
  comInTorso = rot * comInTorso;
  const Vector2f diff = comInTorso.translation.head<2>();

  // if the feet are moving sideways, the maximal forward and backward positions must be reduced
  const float legLength = (engine.walkHipHeight - engine.theRobotDimensions.footHeight);
  const Angle sideSupport = std::atan2(sideL, legLength);
  const Angle sideSwing = std::atan2(sideR, legLength);
  const Angle useAngle = sideSupport > sideSwing ? sideSupport : sideSwing;
  const Rangef clipForward(std::min(engine.clipForward.min * (1.f - useAngle), std::min(leftFoot.translation.x(), rightFoot.translation.x())),
                           std::max(engine.clipForward.max * (1.f - useAngle), std::max(leftFoot.translation.x(), rightFoot.translation.x())));

  // Adjust feet to ensure balance
  walkStepAdjustment.addBalance(leftFoot, rightFoot, t / stepDuration, diff.x(), engine.unstableBackWalkThreshold,
                                engine.desiredFootArea, engine.hipBalanceBackwardFootArea,
                                engine.theFootOffset, engine.maxVelForward, engine.minVelForward, engine.removeAdjustmentVel,
                                engine.comLowPassRatio, clipForward, isLeftPhase,
                                engine.theFootSupport, engine.theFrameInfo, armCompensationTilt,
                                walkKickStep.currentKick == WalkKicks::none && walkState != standing, // should step adjustment be active?
                                engine.unstableWalkThreshold, engine.reduceWalkingSpeedTimeWindow, engine.reduceWalkingSpeedStepAdjustmentSteps,
                                ball, engine.clipAtBallDistanceX);

  if(engine.theFrameInfo.getTimeSince(annotationTimestamp) > 10000 && (std::abs(walkStepAdjustment.lastLeftAdjustmentX) > engine.unstableWalkThreshold || std::abs(walkStepAdjustment.lastRightAdjustmentX) > engine.unstableWalkThreshold))
  {
    annotationTimestamp = engine.theFrameInfo.time;
    ANNOTATION("WalkingEngine", "Step adjusted");
  }
  VERIFY(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f(0.f, 0.f), jointRequest, engine.theRobotDimensions) || SystemCall::getMode() == SystemCall::logFileReplay);
}

void WalkPhase::addGyroBalance(JointRequest& jointRequest)
{
  // Sagittal balance
  Angle balanceAdjustment = walkState == standing ? 0.f : engine.filteredGyroY *
                            (engine.filteredGyroY > 0
                             ? engine.gyroForwardBalanceFactor
                             : engine.gyroBackwardBalanceFactor); // adjust ankle tilt in proportion to filtered gyroY
  jointRequest.angles[isLeftPhase ? Joints::rAnklePitch : Joints::lAnklePitch] += balanceAdjustment;
  PLOT("module:WalkingEngine:Data:sagital", balanceAdjustment.toDegrees());

  // Balance with the hipPitch for one motion phase after a kick phase.
  if(walkKickStep.currentKick != WalkKicks::none || afterWalkKickPhase)
  {
    const Angle balancingValue = engine.filteredGyroY *
                                 (engine.filteredGyroY > 0
                                  ? engine.gyroForwardBalanceFactorHipPitch.y() * walkStepAdjustment.hipBalanceIsSafeBackward
                                  : engine.gyroForwardBalanceFactorHipPitch.x()) * walkStepAdjustment.hipBalanceIsSafeForward;
    PLOT("module:WalkingEngine:Data:hipBalancing", balancingValue.toDegrees());
    jointRequest.angles[isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += balancingValue;
  }

  // The previous steps got adjusted. To prevent that the robot now swings to the opposite side (forward/backward),
  // the hip pitch is used to remove any unwanted velocity
  const float stepAdjustment = std::abs(walkStepAdjustment.lastLeftAdjustmentX) > std::abs(walkStepAdjustment.lastRightAdjustmentX) ?
                               walkStepAdjustment.lastLeftAdjustmentX :
                               walkStepAdjustment.lastRightAdjustmentX;
  if(hipGyroBalance != WalkStepAdjustment::noHipBalance && !(walkKickStep.currentKick != WalkKicks::none || afterWalkKickPhase) &&
     (hipGyroBalance == WalkStepAdjustment::backwardHipBalance && stepAdjustment <= 0.f && walkStepAdjustment.hipBalanceIsSafeBackward > 0.f))
  {
    const Angle balancingValue = engine.filteredGyroY * walkStepAdjustment.hipBalanceIsSafeBackward *
                                 (engine.filteredGyroY > 0
                                  ? 0.f
                                  : engine.gyroForwardBalanceFactorHipPitch.x());
    PLOT("module:WalkingEngine:Data:hipBalancing", balancingValue.toDegrees());
    jointRequest.angles[isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += balancingValue;
  }

  // Lateral balance
  if(walkState == standing && engine.theGroundContactState.contact)
  {
    balanceAdjustment = engine.filteredGyroX * engine.gyroSidewaysBalanceFactor;
    jointRequest.angles[Joints::lAnkleRoll] += balanceAdjustment;
    jointRequest.angles[Joints::rAnkleRoll] += balanceAdjustment;
    PLOT("module:WalkingEngine:Data:lateral", balanceAdjustment.toDegrees());
  }

  // compensate Arms
  jointRequest.angles[Joints::lHipPitch] += armCompensationTilt + std::max(standFactor, 0.f) * engine.standHighTorsoPitch;
  jointRequest.angles[Joints::rHipPitch] += armCompensationTilt + std::max(standFactor, 0.f) * engine.standHighTorsoPitch;

  currentWalkPhaseKneeHipBalance = 0_deg;
  // The robot was tilting backward and is not falling forward, as a result from the pendulum like swinging.
  // To stabilize the robot, the knee and hip pitch are adjusted, to prevent extra momentum resulting from the joints themself.
  if(walkStepAdjustment.kneeHipBalanceCounter > 0 && engine.filteredGyroY < engine.gyroBalanceKneeNegativeGyroAbort && engine.lastFilteredGyroY > engine.gyroBalanceKneeNegativeGyroAbort)
    walkStepAdjustment.kneeHipBalanceCounter = 0;
  if((t > stepDuration || walkStepAdjustment.kneeHipBalanceCounter > 0 || slowDownHipKneeBalanceFactor > 0.f) &&
     walkState == walking && engine.theGroundContactState.contact)
  {
    if(t > stepDuration || walkStepAdjustment.kneeHipBalanceCounter > 0)
      slowDownHipKneeBalanceFactor = 1.f;
    const Rangef balanceRange(0.f, engine.maxGyroBalanceKneeValue);
    float factor = 1.f;
    if(!engine.theGroundContactState.contact)
      factor *= Rangef::ZeroOneRange().limit(1.f - std::min(engine.theFrameInfo.getTimeSince(engine.theFootSupport.footPressure[Legs::left].hasPressure), engine.theFrameInfo.getTimeSince(engine.theFootSupport.footPressure[Legs::right].hasPressure)) / 100.f);
    currentWalkPhaseKneeHipBalance = balanceRange.limit(engine.filteredGyroY * engine.gyroBalanceKneeBalanceFactor) * factor * walkStepAdjustment.hipBalanceIsSafeForward * slowDownHipKneeBalanceFactor;
    jointRequest.angles[isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += currentWalkPhaseKneeHipBalance;
    jointRequest.angles[isLeftPhase ? Joints::rKneePitch : Joints::lKneePitch] += currentWalkPhaseKneeHipBalance;
  }
  jointRequest.angles[!isLeftPhase ? Joints::rHipPitch : Joints::lHipPitch] += lastWalkPhaseKneeHipBalance;
  jointRequest.angles[!isLeftPhase ? Joints::rKneePitch : Joints::lKneePitch] += lastWalkPhaseKneeHipBalance;
  PLOT("module:WalkingEngine:Data:kneeHipBalance", currentWalkPhaseKneeHipBalance.toDegrees());
}

void WalkPhase::calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR, const float footHL, const float footHR, const Angle turn, Pose3f& leftFoot, Pose3f& rightFoot)
{
  leftFoot = Pose3f(0, engine.theRobotDimensions.yHipOffset, 0)
             .translate(forwardL - engine.torsoOffset, sideL, -(engine.walkHipHeight - footHL))
             .rotateZ(turn)
             .rotateY(soleRotationYL)
             .rotateX(soleRotationXL);
  rightFoot = Pose3f(0, -engine.theRobotDimensions.yHipOffset, 0)
              .translate(forwardR - engine.torsoOffset, sideR, -(engine.walkHipHeight - footHR))
              .rotateZ(-turn)
              .rotateY(soleRotationYR)
              .rotateX(soleRotationXR);
}

void WalkingEngine::calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR, const float footHL, const float footHR, const Angle turn, Pose3f& leftFoot, Pose3f& rightFoot)
{
  leftFoot = Pose3f(0, theRobotDimensions.yHipOffset, 0)
             .translate(forwardL - torsoOffset, sideL, -(walkHipHeight - footHL))
             .rotateZ(turn);
  rightFoot = Pose3f(0, -theRobotDimensions.yHipOffset, 0)
              .translate(forwardR  - torsoOffset, sideR, -(walkHipHeight - footHR))
              .rotateZ(-turn);
}

bool WalkPhase::isStandingPossible(float forwardL, float forwardR, float sideL, float sideR, float turnRL, float balanceAdjustmentLeft, float balanceAdjustmentRight) const
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

void WalkPhase::setArms(Pose3f& leftFoot, Pose3f rightFoot, JointRequest& jointRequest, JointRequest& walkArms)
{
  walkArms.angles[Joints::lShoulderPitch] = 90_deg + (leftFoot.translation.x() + engine.torsoOffset + walkStepAdjustment.lastLeftAdjustmentX) * engine.armShoulderPitchFactor / 1000.f;
  walkArms.angles[Joints::lShoulderRoll] = engine.armShoulderRoll + std::abs(leftFoot.translation.y() - engine.theRobotDimensions.yHipOffset) * engine.armShoulderRollIncreaseFactor / 1000.f;
  walkArms.angles[Joints::lElbowYaw] = 0_deg;
  walkArms.angles[Joints::lElbowRoll] = 0_deg;
  walkArms.angles[Joints::lWristYaw] = -90_deg;
  walkArms.angles[Joints::lHand] = 0.f;
  walkArms.angles[Joints::rShoulderPitch] = 90_deg + (rightFoot.translation.x() + engine.torsoOffset + walkStepAdjustment.lastRightAdjustmentX) * engine.armShoulderPitchFactor / 1000.f;
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
    leftArmInterpolationTime = std::max(std::abs(leftArm.angles[Joints::lShoulderPitch]), std::abs(leftArm.angles[Joints::lShoulderRoll] - engine.armShoulderRoll)) / engine.standInterpolationVelocity * 1000.f;
  }
  if(jointRequest.angles[Joints::rShoulderPitch] != JointAngles::ignore)
  {
    MotionUtilities::copy(jointRequest, rightArm, engine.theStiffnessSettings, Joints::firstRightArmJoint, Joints::firstLegJoint);
    rightArmInterpolationStart = engine.theFrameInfo.time;
    rightArmInterpolationTime = std::max(std::abs(rightArm.angles[Joints::rShoulderPitch]), std::abs(rightArm.angles[Joints::rShoulderRoll] - engine.armShoulderRoll)) / engine.standInterpolationVelocity * 1000.f;
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
  armCompensationAfterKick = std::max(0.f, armCompensationAfterKick - Constants::motionCycleTime * 1000.f / engine.baseWalkPeriod);
  armCompensationTilt = (balanced.centerOfMass.x() - withWalkGeneratorArms.centerOfMass.x()) * (1.f - armCompensationAfterKick) * engine.comTiltFactor;

  const RobotModel withHipRot(jointRequest, engine.theRobotDimensions, engine.theMassCalibration);
  leftFoot = withHipRot.soleLeft;
  rightFoot = withHipRot.soleRight;

  PLOT("module:WalkingEngine:armCompensation", armCompensationTilt.toDegrees());
}

void WalkPhase::checkGyroState(const bool ignoreXGyro)
{
  if(engine.theGyroState.timestamp != gyroStateTimestamp)
  {
    gyroStateTimestamp = engine.theGyroState.timestamp;
    const bool xCheck = (std::abs(engine.theGyroState.deviation.x()) < engine.emergencyMaxGyroDeviation && std::abs(engine.theGyroState.mean.x()) < engine.emergencyMaxGyroMean) || ignoreXGyro;
    const bool yCheck = std::abs(engine.theGyroState.deviation.y()) < engine.emergencyMaxGyroDeviation && std::abs(engine.theGyroState.mean.y()) < engine.emergencyMaxGyroMean;
    const bool zCheck = std::abs(engine.theGyroState.deviation.z()) < engine.emergencyMaxZGyroDeviation && std::abs(engine.theGyroState.mean.z()) < engine.emergencyMaxGyroMean;
    if(xCheck && yCheck && zCheck)
      robotIsNotMoving++;
    else
      robotIsNotMoving = 0;
  }
}

void WalkPhase::applyWalkKickLongKickOffset(JointRequest& jointRequest, const float time)
{
  if(walkKickStep.currentKick == WalkKicks::none)
    return;
  for(WalkKickStep::LongKickParams param : walkKickStep.longKickParams)
  {
    const Joints::Joint joint = isLeftPhase ? param.joint : Joints::mirror(param.joint);
    const float useTime = std::max(0.f, time / stepDuration < param.middleRatio ?  time - stepDuration* param.minRatio : time - stepDuration * param.middleRatio);
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

void WalkPhase::adjustRequestByMeasuredPosition(const bool isLeftPhase, const float time, const float duration, const WalkKickStep& walkKickStep)
{
  if(walkKickStep.longKickParams.size() > 0)
  {
    // Get last joint request without arm compensation
    JointRequest request;
    request.angles = engine.theJointAngles.angles;
    request.angles[Joints::lHipPitch] -= armCompensationTilt;
    request.angles[Joints::rHipPitch] -= armCompensationTilt;

    // Get resulting swing foot pose and clip it
    const RobotModel model(request, engine.theRobotDimensions, engine.theMassCalibration);

    // Update swing foot variables
    if((isLeftPhase && walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::measured) || (!isLeftPhase && walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::measured))
    {
      soleRotationYL = model.soleLeft.rotation.getYAngle();
      soleRotationXL = model.soleLeft.rotation.getXAngle();
      forwardL0 = model.soleLeft.translation.x() + engine.torsoOffset;
      footHL0 = engine.walkHipHeight + model.soleLeft.translation.z() - (maxFootHeight * parabolicFootHeight(time / duration));
    }
    if((!isLeftPhase && walkKickStep.overrideOldSwingFoot == WalkKickStep::OverrideFoot::measured) || (isLeftPhase && walkKickStep.overrideOldSupportFoot == WalkKickStep::OverrideFoot::measured))
    {
      soleRotationYR = model.soleRight.rotation.getYAngle();
      soleRotationXR = model.soleRight.rotation.getXAngle();
      forwardR0 = model.soleRight.translation.x() + engine.torsoOffset;
      footHR0 = engine.walkHipHeight + model.soleRight.translation.z() - (maxFootHeight * parabolicFootHeight(time / duration));
    }
  }
}

void WalkPhase::calculateBallPosition()
{
  const Pose3f supportInTorso3D = engine.theTorsoMatrix * (isLeftPhase ? engine.theRobotModel.soleRight : engine.theRobotModel.soleLeft);
  const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
  const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -engine.theRobotDimensions.yHipOffset : engine.theRobotDimensions.yHipOffset);
  const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * engine.theOdometryData.inverse() * engine.theMotionRequest.odometryData;
  ball = scsCognition * engine.theMotionRequest.ballEstimate.position;
  if(ball.squaredNorm() > sqr(engine.clipAtBallDistance) || ball.x() <= 0.f)
    ball = Vector2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
}
