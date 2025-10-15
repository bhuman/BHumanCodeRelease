/**
 * @file RLWalkingEngine.h
 *
 * @Author Philip Reichenberg
 */

#include "RLWalkingEngine.h"
#include "Platform/SystemCall.h"
#include <filesystem>

MAKE_MODULE(RLWalkingEngine);

RLWalkingEngine::RLWalkingEngine():
  network(&Global::getAsmjitRuntime())
{
  compile(false);
  // https://github.com/BoosterRobotics/booster_gym/blob/main/deploy/configs/T1.yaml#L19
  offset.angles[Joints::lHipPitch] = -0.2f;
  offset.angles[Joints::lKneePitch] = 0.4f;
  offset.angles[Joints::lAnklePitch] = -0.25f;
  offset.angles[Joints::rHipPitch] = -0.2f;
  offset.angles[Joints::rKneePitch] = 0.4f;
  offset.angles[Joints::rAnklePitch] = -0.25f;
  while(!lastMeasurements.full())
    lastMeasurements.push_front(JointAngles());

  const DummyPhase dummy(MotionPhase::playDead);
  RLWalkPhase phase(*this, Pose2f(), dummy);

  const float stepDuration = configuredParameters.baseWalkPeriod / 1000.f;
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x() / 2.f, configuredParameters.maxSpeed.translation.y()));
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x(), configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x(), -configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(configuredParameters.maxSpeed.translation.x() / 2.f, -configuredParameters.maxSpeed.translation.y()));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards / 2.f, -configuredParameters.maxSpeed.translation.y()));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards, -configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards, configuredParameters.maxSpeed.translation.y() / 2.f));
  translationPolygon.emplace_back(stepDuration * Vector2f(-configuredParameters.maxSpeedBackwards / 2.f, configuredParameters.maxSpeed.translation.y()));

  boosterJoints = { Joints::lHipPitch,
                    Joints::lHipRoll,
                    Joints::lHipYaw,
                    Joints::lKneePitch,
                    Joints::lAnklePitch,
                    Joints::lAnkleRoll,
                    Joints::rHipPitch,
                    Joints::rHipRoll,
                    Joints::rHipYaw,
                    Joints::rKneePitch,
                    Joints::rAnklePitch,
                    Joints::rAnkleRoll
                  };

  boosterWaistJoints = { Joints::waistYaw };
  boosterWaistJoints.insert(boosterWaistJoints.end(), boosterJoints.begin(), boosterJoints.end());

  naoJoints = { Joints::lHipYawPitch,
                Joints::lHipRoll,
                Joints::lHipPitch,
                Joints::lKneePitch,
                Joints::lAnklePitch,
                Joints::lAnkleRoll,
                Joints::rHipYawPitch,
                Joints::rHipRoll,
                Joints::rHipPitch,
                Joints::rKneePitch,
                Joints::rAnklePitch,
                Joints::rAnkleRoll
              };
}

void RLWalkingEngine::compile(bool output)
{
  if(output)
  {
    if(!std::filesystem::exists(modelPath + walkNeuronalNetworkParameters.modelName))
    {
      OUTPUT_ERROR("File " << modelPath << walkNeuronalNetworkParameters.modelName << " does not exist");
      return;
    }
  }
  else
    ASSERT(std::filesystem::exists(modelPath + walkNeuronalNetworkParameters.modelName));

  network.compile(Model(modelPath + walkNeuronalNetworkParameters.modelName));
  ASSERT(network.valid());

  // Input shape: (47)
  ASSERT(network.numOfInputs() == 1);
  ASSERT(network.input(0).rank() == 1); // (Batch, Time, Features)
  ASSERT(network.input(0).dims(0) == walkNeuronalNetworkParameters.numInput);

  // Output shape: (1, 11)
  ASSERT(network.numOfOutputs() == 1);
  ASSERT(network.output(0).rank() == 1); // (Batch, Time, Features)
  ASSERT(network.output(0).dims(0) == walkNeuronalNetworkParameters.numOutput); // TODO
}

void RLWalkingEngine::update(WalkStepData& walkStepData)
{
  lastMeasurements.push_front(theJointAngles);

  walkStepData.updateCounter = [&walkStepData](const bool)
  {
    walkStepData.usedPredictedSwitch = 0; // Unused
  };
  walkStepData.updateWalkValues = [this, &walkStepData](const Pose2f& stepTarget, const float, const bool isLeftPhase)
  {
    walkStepData.isLeftPhase = isLeftPhase;
    walkStepData.stepTarget = stepTarget;
    walkStepData.stepDuration = 200; // static
    walkStepData.lastUpdate = theFrameInfo.time;
  };

  walkStepData.yHipOffset = 100.f; /// Unknown
}

void RLWalkingEngine::update(StandGenerator& standGenerator)
{
  standGenerator.createPhase = [this](const MotionRequest&, const MotionPhase& lastPhase)
  {
    return std::make_unique<RLWalkPhase>(*this, Pose2f(), lastPhase);
  };
}

void RLWalkingEngine::update(WalkingEngineOutput& walkingEngineOutput)
{
  Pose2f useSpeed;
  useSpeed.rotation = configuredParameters.maxSpeed.rotation;
  useSpeed.translation = configuredParameters.maxSpeed.translation;
  const float useSpeedBackwards = configuredParameters.maxSpeedBackwards;

  walkingEngineOutput.maxSpeed = useSpeed;
  walkingEngineOutput.maxSpeedBackwards = useSpeedBackwards;
  walkingEngineOutput.walkStepDuration = configuredParameters.baseWalkPeriod / 1000.f;

  walkingEngineOutput.maxStepSize = Pose2f(useSpeed.rotation * walkingEngineOutput.walkStepDuration,
                                           useSpeed.translation.x() * walkingEngineOutput.walkStepDuration,
                                           useSpeed.translation.y() * walkingEngineOutput.walkStepDuration * 2.f);
  walkingEngineOutput.maxBackwardStepSize = useSpeedBackwards * walkingEngineOutput.walkStepDuration;

  walkingEngineOutput.maxPossibleStepSize = Pose2f(configuredParameters.maxSpeed.rotation * walkingEngineOutput.walkStepDuration,
                                                   configuredParameters.maxSpeed.translation.x() * walkingEngineOutput.walkStepDuration,
                                                   configuredParameters.maxSpeed.translation.y() * walkingEngineOutput.walkStepDuration * 2.f);
  walkingEngineOutput.maxPossibleBackwardStepSize = configuredParameters.maxSpeedBackwards * walkingEngineOutput.walkStepDuration;

  walkingEngineOutput.energyEfficientWalkSpeed = configuredParameters.maxSpeed;
  walkingEngineOutput.energyEfficientWalkSpeed.rotation *= 0.9f; // prevent possible divisions by 0 in behavior
  walkingEngineOutput.energyEfficientWalkSpeed.translation *= 0.9f;
  walkingEngineOutput.noEfficientWalkSpeed = configuredParameters.maxSpeed;
}

void RLWalkingEngine::update(WalkGenerator& walkGenerator)
{
  walkGenerator.createPhase = [this](const Pose2f& step, const MotionPhase& lastPhase, float)->std::unique_ptr<MotionPhase>
  {
    return std::make_unique<RLWalkPhase>(*this, step, lastPhase);
  };

  walkGenerator.createPhaseWithNextPhase = [this](const WalkKickStep& walkKickStep, const MotionPhase& lastPhase, const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback, float)->std::unique_ptr<MotionPhase>
  {
    return std::make_unique<RLWalkPhase>(*this, walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTarget, lastPhase, false, createNextPhaseCallback, walkKickStep);
  };

  walkGenerator.isNextLeftPhase = [this](const MotionPhase& lastPhase, const Pose2f& stepTarget)
  {
    switch(lastPhase.type)
    {
      case MotionPhase::walk:
      {
        const auto& lastWalkPhase = static_cast<const RLWalkPhase&>(lastPhase);
        return lastWalkPhase.tBase < 0.44f || lastWalkPhase.tBase > 0.94f;
      }
      case MotionPhase::kick:
      case MotionPhase::stand:
      {
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

    const auto& lastWalkPhase = static_cast<const RLWalkPhase&>(lastPhase);
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
    const auto& lastWalkPhase = static_cast<const RLWalkPhase&>(lastPhase);
    step = lastWalkPhase.step;
    walkKickStep = lastWalkPhase.walkKickStep;
  };

  walkGenerator.isWalkDelayPossible = [](const MotionPhase&, const float, const Pose2f&, const bool)
  {
    return false;
  };

  walkGenerator.getRotationRange = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio)
  {
    const Angle rotation = configuredParameters.maxSpeed.rotation;
    const float innerTurn = 2.f * 0.5f * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - 0.5f) * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
    return Rangea(isLeftPhase ? -innerTurn : -outerTurn, isLeftPhase ? outerTurn : innerTurn);
  };

  walkGenerator.getStepRotationRangeOther = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio, Vector2f step,
                                                   const bool isFastWalk, const std::vector<Vector2f>& translationPolygon,
                                                   const bool ignoreXTranslation, const bool isMaxPossibleStepSize)
  {
    ASSERT(translationPolygon.size() >= 4);
    if(translationPolygon.size() < 4)
      return Rangea(0_deg, 0_deg);

    const float durationValue = configuredParameters.baseWalkPeriod / 1000.f;
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

    const Angle rotation = configuredParameters.maxSpeed.rotation;
    const float innerTurn = 2.f * stepSizeParameters.insideTurnRatio * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
    const float outerTurn = 2.f * (1.f - stepSizeParameters.insideTurnRatio) * rotation * std::abs(walkSpeedRatio.rotation) * configuredParameters.baseWalkPeriod / 1000.f;
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

    Vector2f frontLeft, backRight;
    const Pose2f lastStep = lastPhase.type == MotionPhase::walk ? static_cast<const RLWalkPhase&>(lastPhase).step : Pose2f();
    const Vector2f maxStepSizeChange = commonSpeedParameters.maxAcceleration * configuredParameters.baseWalkPeriod / 1000.f;
    const Vector2f maxStepSizeChangeToZero = commonSpeedParameters.maxDeceleration * configuredParameters.baseWalkPeriod / 1000.f;
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
    Vector2f useSpeed = configuredParameters.maxSpeed.translation;
    const float useSpeedBackwards = configuredParameters.maxSpeedBackwards;

    // Limit to maximum speed (which is influenced by the rotation).
    // When the arms are on the back, the robot will unintentionally turn more in each step.
    // Allow more rotation + translation, so the robot does not walk that much slower
    const Vector2f stepSizeFactor = getStepSizeFactor(rotation, useFastWalk, isLeftPhase, 10_deg, useWalkSpeedRatio.translation);

    backRight.x() = std::max(backRight.x(), stepSizeFactor.x() * -useSpeedBackwards * configuredParameters.baseWalkPeriod / 1000.f);
    backRight.y() = std::max(backRight.y(), stepSizeFactor.y() * -2.f * useSpeed.y() * configuredParameters.baseWalkPeriod / 1000.f);
    frontLeft.x() = std::min(frontLeft.x(), stepSizeFactor.x() * useSpeed.x() * configuredParameters.baseWalkPeriod / 1000.f);
    frontLeft.y() = std::min(frontLeft.y(), stepSizeFactor.y() * 2.f * useSpeed.y() * configuredParameters.baseWalkPeriod / 1000.f);

    // Make sure some minimum exists
    const float useMinXBackwardTranslation = stepSizeParameters.minXBackwardTranslationFastRange.max;
    frontLeft.x() = std::max(stepSizeParameters.minXForwardTranslationFast * useWalkSpeedRatio.translation.x(), frontLeft.x());
    backRight.x() = std::min(useMinXBackwardTranslation * useWalkSpeedRatio.translation.x(), backRight.x());

    // Step size in x translation has a min size
    const float maxMinStepX = std::min(stepSizeParameters.minXTranslationStep, (useWalkSpeedRatio.translation.x() >= 0.f ? useSpeed.x() : useSpeedBackwards) * configuredParameters.baseWalkPeriod / 1000.f * std::abs(useWalkSpeedRatio.translation.x()) + 0.01f);

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

  walkGenerator.generateTranslationPolygon = [this](const bool isLeftPhase, const Angle rotation, const Pose2f& walkSpeedRatio,
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

  walkGenerator.getStartOffsetOfNextWalkPhase = [this](const MotionPhase&)
  {
    Pose2f left;
    Pose2f right;
    Pose2f lastStep;

    const RobotModel lastRobotModel(theJointRequest, theRobotDimensions, theMassCalibration);
    left.translate(lastRobotModel.soleLeft.translation.head<2>()).rotate(lastRobotModel.soleLeft.rotation.getZAngle()); // Wrong, because the 0 position is unknown
    right.translate(lastRobotModel.soleRight.translation.head<2>()).rotate(lastRobotModel.soleRight.rotation.getZAngle());

    return std::make_tuple(left, right, lastStep);
  };

  walkGenerator.getLastStepChange = [](const MotionPhase&)
  {
    return Pose2f();
  };

  walkGenerator.getLastStepChangeWithOffsets = [&](const MotionPhase&)
  {
    return std::make_tuple(Pose2f(), Pose2f(), Pose2f());
  };

  walkGenerator.wasLastPhaseInWalkKick = [](const MotionPhase& lastPhase)
  {
    if(lastPhase.type == MotionPhase::walk)
    {
      const auto& lastWalkPhase = static_cast<const RLWalkPhase&>(lastPhase);
      return lastWalkPhase.walkKickStep.currentKick != WalkKicks::none;
    }
    return false;
  };
}

Rangea RLWalkingEngine::getMaxRotationToStepFactor(const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& stepRatio)
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

Vector2f RLWalkingEngine::getStepSizeFactor(const Angle rotation, const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& walkSpeedRatio)
{
  const Angle useReduceFactor = std::max(customReduceOffset, stepSizeParameters.reduceTranslationFromRotation.x());
  const float tFactorX = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceFactor) / (stepSizeParameters.noTranslationFromRotation.x() - useReduceFactor))));
  const float reduceTransitionY = !isFastWalk ? stepSizeParameters.noTranslationFromRotation.y() : ((isLeftPhase && rotation > 0_deg) || (!isLeftPhase && rotation < 0_deg) ? stepSizeParameters.noTranslationYFromRotationFastOuter : stepSizeParameters.noTranslationYFromRotationFastInner);
  const float useReduceTranslationFromRotation = !isFastWalk ? stepSizeParameters.reduceTranslationFromRotation.y() : stepSizeParameters.reduceTranslationYFromRotationFast;
  const float tFactorY = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - useReduceTranslationFromRotation) / (reduceTransitionY - useReduceTranslationFromRotation))));
  return Vector2f(std::min(tFactorX, std::abs(walkSpeedRatio.x())), std::min(tFactorY, std::abs(walkSpeedRatio.y())));
}

void RLWalkingEngine::filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal)
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

void RLWalkingEngine::generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool)
{
  ASSERT(!translationPolygon.empty());
  const std::vector<Vector2f>& original = translationPolygon;
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

RLWalkPhase::RLWalkPhase(RLWalkingEngine& engine, Pose2f useStepTarget, const MotionPhase& lastPhase, const bool,
                         const WalkGenerator::CreateNextPhaseCallback& callBack,
                         const WalkKickStep& walkKickStep):
  MotionPhase(MotionPhase::walk),
  walkKickStep(walkKickStep),
  engine(engine),
  callBack(callBack)
{
  step = useStepTarget;
  slowWalkStart = engine.theFrameInfo.time > static_cast<unsigned>(engine.walkNeuronalNetworkParameters.timeLowWalkSpeedForStand) ? engine.theFrameInfo.time - engine.walkNeuronalNetworkParameters.timeLowWalkSpeedForStand : 0;
  tBase = engine.motionCycleTime;

  lastTarget.angles = engine.theJointRequest.angles;
  lastModelRequest = engine.theFrameInfo.time;
  if(lastPhase.type == MotionPhase::walk || lastPhase.type == MotionPhase::stand)
  {
    const auto& lastWalkPhase = static_cast<const RLWalkPhase&>(lastPhase);
    tBase = lastWalkPhase.tBase;
    lastTarget = lastWalkPhase.lastTarget;
    lastModelRequest = lastWalkPhase.lastModelRequest;
    nextTarget = lastWalkPhase.nextTarget;
    lastWalking = lastWalkPhase.lastWalking;
    if(lastPhase.type == MotionPhase::stand)
    {
      tBase = engine.theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget) ? 0.f : 0.5f;
    }
    slowWalkStart = lastWalkPhase.slowWalkStart;
  }
  else
    lastWalking = engine.theFrameInfo.time > 2000 ? engine.theFrameInfo.time - 2000 : 0;

  if(!(std::abs(step.rotation) < engine.walkNeuronalNetworkParameters.slowWalkStepSpeed.rotation &&
       std::abs(step.translation.x()) < engine.walkNeuronalNetworkParameters.slowWalkStepSpeed.translation.x() &&
       std::abs(step.translation.y()) < engine.walkNeuronalNetworkParameters.slowWalkStepSpeed.translation.y()))
    slowWalkStart = engine.theFrameInfo.time;

  // Standing is only allowed after a short time to prevent badly falling due to leftover momentum
  if(step == Pose2f() && engine.theFrameInfo.getTimeSince(slowWalkStart) < engine.walkNeuronalNetworkParameters.timeLowWalkSpeedForStand && slowWalkStart != 0)
    step = Pose2f(0.001f, 0.f);

  if(step == Pose2f())
  {
    type = MotionPhase::stand;
    tBase = 0.f;
  }

  // Set walk values
  engine.theWalkStepData.updateWalkValues(step, 0.2f, isLeftPhase);
  //lastTarget = engine.theJointAngles;
  lastModelRequest = engine.theFrameInfo.time;
  getNextTargetRequest(nextTarget);

  isLeftPhase = type == MotionPhase::stand || tBase < 0.5f;

  tWalk = 0.f;
  if(type == MotionPhase::walk)
    lastWalking = engine.theFrameInfo.time;
  else if(engine.theFrameInfo.getTimeSince(lastWalking) >= 2000)
  {
    FOREACH_ENUM(Joints::Joint, joint)
    {
      requestAtStandStart.angles[joint] = engine.theJointRequest.stiffnessData.stiffnesses[joint] > 0 ?
                                          engine.theJointRequest.angles[joint] :
                                          engine.theJointAngles.angles[joint];
    }
  }

  frequency = rawFrequency = engine.frequencyParameters.base;
}

void RLWalkPhase::getNextTargetRequest(JointAngles& target)
{
  float* input = engine.network.input(0).data();

  const Vector3f gravity = engine.theInertialData.orientation3D.inverse() * Vector3f(0.f, 0.f, -1.f);

  // gravity
  *input++ = gravity.x();
  *input++ = gravity.y();
  *input++ = gravity.z();

  // angular momentum
  *input++ = engine.theInertialData.gyro.x();
  *input++ = engine.theInertialData.gyro.y();
  *input++ = engine.theInertialData.gyro.z();

  // walk command
  if(type == MotionPhase::walk)
  {
    *input++ = step.translation.x() / (engine.configuredParameters.baseWalkPeriod / 1000.f) / 1000.f;
    *input++ = step.translation.y() / (engine.configuredParameters.baseWalkPeriod / 1000.f) / 1000.f;
    *input++ = step.rotation / (engine.configuredParameters.baseWalkPeriod / 1000.f);
  }
  else
  {
    *input++ = 0.f;
    *input++ = 0.f;
    *input++ = 0.f;
  }

  // walk cycle
  *input++ = type == MotionPhase::walk ? std::cos(2.f * Constants::pi * tBase) : 0.f;
  *input++ = type == MotionPhase::walk ? std::sin(2.f * Constants::pi * tBase) : 0.f;

  // joint sequence
  std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();

  // Measurements
  for(Joints::Joint j : jointList)
    *input++ = engine.theJointAngles.angles[j] - engine.offset.angles[j];

  // Velocities
  for(Joints::Joint j : jointList)
    *input++ = engine.theJointAngles.velocity[j] * engine.walkNeuronalNetworkParameters.velocityFactor;

  // Last Requests
  for(Joints::Joint j : jointList)
    *input++ = engine.theJointRequest.angles[j] - engine.offset.angles[j];

  if(engine.frequencyParameters.active)
    *input++ = rawFrequency;

  if(engine.ballNetworkParameters.active)
  {
    // Ball position (3D)
    *input++ = 0; // x
    *input++ = 0; // y
    *input++ = 0; // z

    // Ball velocity (3D)
    *input++ = 0; // x
    *input++ = 0; // y
    *input++ = 0; // z

    // Ball direction as sin and cos
    *input++ = 0; // sin(direction), 0 for direction 0deg
    *input++ = 1; // cos(direction), 1 for direction 0deg

    // Ball range
    *input++ = 0;

    // Play ball (1) or not play ball (0)
    *input++ = 0;

    // Just walk (1) or not just walk (0)
    *input++ = 1;
  }

  // Run network.
  STOPWATCH("module:RLWalkingEngine:apply")
    engine.network.apply();

  // Get next request
  const float* output = engine.network.output(0).data();
  for(Joints::Joint j : jointList)
    target.angles[j] = engine.walkNeuronalNetworkParameters.actionClipRange.limit(*output++) + engine.offset.angles[j];

  if(engine.frequencyParameters.active)
  {
    rawFrequency = Rangef::OneRange().limit(*output++);
    frequency = engine.frequencyParameters.clipRange.limit(rawFrequency) + engine.frequencyParameters.base;
  }
}

void RLWalkPhase::update()
{
  if(lastModelRequest > engine.theFrameInfo.time)
    lastModelRequest = engine.theFrameInfo.time;
  tWalk += engine.motionCycleTime;
  tBase = std::fmod(tBase + engine.motionCycleTime * frequency * (type == MotionPhase::walk ? 1.f : 0.f), 1.f);
  if(type == MotionPhase::stand && engine.theFrameInfo.getTimeSince(lastModelRequest) >= 20)
  {
    lastModelRequest = engine.theFrameInfo.time;

    getNextTargetRequest(nextTarget);
  }
}

bool RLWalkPhase::isDone(const MotionRequest& motionRequest) const
{
  // Does not work with WalkKickEngine
  return type == MotionPhase::stand ? motionRequest.motion != MotionRequest::stand // switch from stand to walk
         : engine.theFrameInfo.getTimeSince(lastModelRequest) >= 20 && tWalk > engine.motionCycleTime;
}

void RLWalkPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  jointRequest.stiffnessData.stiffnesses.fill(100);

  jointRequest.angles[Joints::lShoulderPitch] = engine.armParameters.armShoulderPitch + engine.theRobotModel.soleLeft.translation.x() * engine.armParameters.armShoulderPitchFactor / 1000.f;
  jointRequest.angles[Joints::lShoulderRoll] = engine.armParameters.armShoulderRoll + std::abs(engine.theRobotModel.soleLeft.translation.y()) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  jointRequest.angles[Joints::lElbowYaw] = engine.armParameters.armElbowYaw;
  jointRequest.angles[Joints::lElbowRoll] = 0_deg;
  jointRequest.angles[Joints::lWristYaw] = -90_deg;
  jointRequest.angles[Joints::lHand] = 0.f;
  jointRequest.angles[Joints::rShoulderPitch] = engine.armParameters.armShoulderPitch + engine.theRobotModel.soleRight.translation.x() * engine.armParameters.armShoulderPitchFactor / 1000.f;
  jointRequest.angles[Joints::rShoulderRoll] = -engine.armParameters.armShoulderRoll - std::abs(engine.theRobotModel.soleRight.translation.y()) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  jointRequest.angles[Joints::rElbowYaw] = -engine.armParameters.armElbowYaw;
  jointRequest.angles[Joints::rElbowRoll] = 0_deg;
  jointRequest.angles[Joints::rWristYaw] = 90_deg;
  jointRequest.angles[Joints::rHand] = 0.f;

  jointRequest.angles[Joints::waistYaw] = 0_deg;

  if(SystemCall::getMode() != SystemCall::simulatedRobot && engine.theFrameInfo.getTimeSince(lastWalking) >= 2000 && engine.theGameState.playerState == GameState::PlayerState::penalizedManual)
  {
    for(std::size_t j = Joints::firstArmJoint; j < Joints::firstLegJoint; j++)
      standTarget.angles[j] = jointRequest.angles[j];
    const float ratio = Rangef::ZeroOneRange().limit((engine.theFrameInfo.getTimeSince(lastWalking) - 2000) / 2000.f);
    for(std::size_t j = Joints::firstArmJoint; j < Joints::numOfJoints; j++)
      jointRequest.angles[j] = standTarget.angles[j] * ratio + (1.f - ratio) * requestAtStandStart.angles[j];
  }
  else
  {
    std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();
    for(Joints::Joint j : jointList)
    {
      lastTarget.angles[j] = nextTarget.angles[j];
      jointRequest.angles[j] = lastTarget.angles[j];
    }
    standTarget = nextTarget;
    requestAtStandStart.angles = jointRequest.angles;
    if(SystemCall::getMode() != SystemCall::simulatedRobot && type == MotionPhase::stand)
    {
      jointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = 50;
      jointRequest.stiffnessData.stiffnesses[Joints::lAnkleRoll] = 50;
      jointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = 50;
      jointRequest.stiffnessData.stiffnesses[Joints::rAnkleRoll] = 50;
    }
  }

  motionInfo.isMotionStable = true;
  motionInfo.isWalkPhaseInWalkKick = walkKickStep.currentKick != WalkKicks::none;
  const float stepDuration = engine.configuredParameters.baseWalkPeriod / 1000.f;
  motionInfo.speed = type == MotionPhase::walk ? Pose2f(step.rotation / stepDuration, step.translation.x() / stepDuration, step.translation.x() / stepDuration) : Pose2f(); // TODO

  if(engine.theFrameInfo.getTimeSince(lastWalking) >= 2000)
    odometryOffset = Pose2f(0_deg, 0.f, 0.f);
  else
    odometryOffset = engine.theOdometryDataPreview.odometryChange;
}

std::unique_ptr<MotionPhase> RLWalkPhase::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  // There must be another walk phase if the robot should transition into something other than walking and the feet are not next to each other.
  if(defaultNextPhase.type != MotionPhase::walk && type != MotionPhase::stand)
    return std::make_unique<RLWalkPhase>(engine, Pose2f(), *this);
  if(std::unique_ptr<MotionPhase> nextPhase; callBack && (nextPhase = callBack(*this, this->walkKickStep)))
    return nextPhase;
  return std::unique_ptr<MotionPhase>();
}

std::vector<Joints::Joint> RLWalkPhase::getBoosterLegJointSequence()
{
  if(Global::getSettings().robotType != Settings::nao)
  {
    if(engine.walkNeuronalNetworkParameters.useWaist)
      return engine.boosterWaistJoints;
    return engine.boosterJoints;
  }
  return engine.naoJoints;
}
