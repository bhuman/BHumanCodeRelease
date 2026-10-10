/**
 * @file WalkingEngine.h
 *
 * @Author Philip Reichenberg
 */

#include "WalkingEngine.h"
#include "Debugging/Plot.h"
#include "Math/Rotation.h"
#include "Platform/SystemCall.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/KickLengthConverter.h"
#include <filesystem>

MAKE_MODULE(WalkingEngine);

WalkingEngine::WalkingEngine():
  walkPolicy(&Global::getAsmjitRuntime()),
  kickPolicy(&Global::getAsmjitRuntime())
{
  compile();

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

  // https://github.com/BoosterRobotics/booster_gym/blob/main/deploy/configs/T1.yaml#L19
  offset.angles[Joints::lHipPitch] = -0.2f;
  offset.angles[Joints::lKneePitch] = 0.4f;
  offset.angles[Joints::lAnklePitch] = -0.25f;
  offset.angles[Joints::rHipPitch] = -0.2f;
  offset.angles[Joints::rKneePitch] = 0.4f;
  offset.angles[Joints::rAnklePitch] = -0.25f;

  resetHistoryData();

  const DummyPhase dummy(MotionPhase::playDead);
  WalkPhase phase(*this, Pose2f(), dummy);

  const auto& fastSpeed = configuredParameters.fastWalkSpeed;
  const float maxBackwardAtMaxSide = std::min(fastSpeed.maxSpeed.translation.x(), std::min(fastSpeed.maxSpeedBackwards, configuredParameters.maxForwardAtMaxSide));
  const float maxForwardAtMaxSide = std::min(fastSpeed.maxSpeed.translation.x(), configuredParameters.maxForwardAtMaxSide);
  const float maxSideAtMaxForward = std::min(fastSpeed.maxSpeed.translation.y(), configuredParameters.maxSideAtMaxForward);
  translationPolygon.emplace_back(Vector2f(maxForwardAtMaxSide, fastSpeed.maxSpeed.translation.y()));
  translationPolygon.emplace_back(Vector2f(fastSpeed.maxSpeed.translation.x(), maxSideAtMaxForward));
  translationPolygon.emplace_back(Vector2f(fastSpeed.maxSpeed.translation.x(), -maxSideAtMaxForward));
  translationPolygon.emplace_back(Vector2f(maxForwardAtMaxSide, -fastSpeed.maxSpeed.translation.y()));
  translationPolygon.emplace_back(Vector2f(-maxBackwardAtMaxSide, -fastSpeed.maxSpeed.translation.y()));
  translationPolygon.emplace_back(Vector2f(-fastSpeed.maxSpeedBackwards, -maxSideAtMaxForward));
  translationPolygon.emplace_back(Vector2f(-fastSpeed.maxSpeedBackwards, maxSideAtMaxForward));
  translationPolygon.emplace_back(Vector2f(-maxBackwardAtMaxSide, fastSpeed.maxSpeed.translation.y()));

  const auto& maxSpeedMP = configuredParameters.maxPossibleSpeed;
  const float maxBackwardAtMaxSideMP = std::min(maxSpeedMP.maxSpeed.translation.x(), std::min(maxSpeedMP.maxSpeedBackwards, configuredParameters.maxForwardAtMaxSide));
  const float maxForwardAtMaxSideMP = std::min(maxSpeedMP.maxSpeed.translation.x(), configuredParameters.maxForwardAtMaxSide);
  const float maxSideAtMaxForwardMP = std::min(maxSpeedMP.maxSpeed.translation.y(), configuredParameters.maxSideAtMaxForward);
  translationPolygonMaxPossible.emplace_back(Vector2f(maxForwardAtMaxSideMP, maxSpeedMP.maxSpeed.translation.y()));
  translationPolygonMaxPossible.emplace_back(Vector2f(maxSpeedMP.maxSpeed.translation.x(), maxSideAtMaxForwardMP));
  translationPolygonMaxPossible.emplace_back(Vector2f(maxSpeedMP.maxSpeed.translation.x(), -maxSideAtMaxForwardMP));
  translationPolygonMaxPossible.emplace_back(Vector2f(maxForwardAtMaxSideMP, -maxSpeedMP.maxSpeed.translation.y()));
  translationPolygonMaxPossible.emplace_back(Vector2f(-maxBackwardAtMaxSideMP, -maxSpeedMP.maxSpeed.translation.y()));
  translationPolygonMaxPossible.emplace_back(Vector2f(-maxSpeedMP.maxSpeedBackwards, -maxSideAtMaxForwardMP));
  translationPolygonMaxPossible.emplace_back(Vector2f(-maxSpeedMP.maxSpeedBackwards, maxSideAtMaxForwardMP));
  translationPolygonMaxPossible.emplace_back(Vector2f(-maxBackwardAtMaxSideMP, maxSpeedMP.maxSpeed.translation.y()));

  configuredParameters.normalWalkSpeed.maxSpeed.translation.x() = std::min(fastSpeed.maxSpeed.translation.x() * 0.99f, configuredParameters.normalWalkSpeed.maxSpeed.translation.x());
  configuredParameters.normalWalkSpeed.maxSpeed.translation.y() = std::min(fastSpeed.maxSpeed.translation.y() * 0.99f, configuredParameters.normalWalkSpeed.maxSpeed.translation.y());
  configuredParameters.normalWalkSpeed.maxSpeed.rotation = std::min(fastSpeed.maxSpeed.rotation * 0.99f, static_cast<float>(configuredParameters.normalWalkSpeed.maxSpeed.rotation));

  configuredParameters.slowWalkSpeed.maxSpeed.translation.x() = std::min(configuredParameters.normalWalkSpeed.maxSpeed.translation.x() * 0.99f, configuredParameters.slowWalkSpeed.maxSpeed.translation.x());
  configuredParameters.slowWalkSpeed.maxSpeed.translation.y() = std::min(configuredParameters.normalWalkSpeed.maxSpeed.translation.y() * 0.99f, configuredParameters.slowWalkSpeed.maxSpeed.translation.y());
  configuredParameters.slowWalkSpeed.maxSpeed.rotation = std::min(configuredParameters.normalWalkSpeed.maxSpeed.rotation * 0.99f, static_cast<float>(configuredParameters.slowWalkSpeed.maxSpeed.rotation));

  ASSERT(configuredParameters.slowWalkSpeed.maxSpeed.translation.x() < configuredParameters.normalWalkSpeed.maxSpeed.translation.x());
  ASSERT(configuredParameters.slowWalkSpeed.maxSpeed.translation.y() < configuredParameters.normalWalkSpeed.maxSpeed.translation.y());
  ASSERT(configuredParameters.slowWalkSpeed.maxSpeed.rotation < configuredParameters.normalWalkSpeed.maxSpeed.rotation);

  ASSERT(configuredParameters.normalWalkSpeed.maxSpeed.translation.x() < fastSpeed.maxSpeed.translation.x());
  ASSERT(configuredParameters.normalWalkSpeed.maxSpeed.translation.y() < fastSpeed.maxSpeed.translation.y());
  ASSERT(configuredParameters.normalWalkSpeed.maxSpeed.rotation < fastSpeed.maxSpeed.rotation);
}

void WalkingEngine::resetHistoryData()
{
  historyBuffer.clear();
  if(theJointAngles.angles[0] != JointAngles::off && theJointAngles.angles[0] != JointAngles::ignore && theJointAngles.timestamp > 0)
  {
    for(std::size_t i = 0; i < historyBuffer.capacity(); i++)
      updateHistoryData(theJointRequest, true);
  }
  else
  {
    HistoryData nextDataPoint;
    nextDataPoint.gravity = Vector3f(0.f, 0.f, -1.f);
    nextDataPoint.gyro = Vector3a::Zero();;
    nextDataPoint.measuredAngles = JointAngles();
    nextDataPoint.lastActions = JointAngles();

    for(std::size_t i = 0; i < historyBuffer.capacity(); i++)
      historyBuffer.push_front(nextDataPoint);
  }
}

void WalkingEngine::updateHistoryData(const JointAngles& lastAction, const bool init)
{
  HistoryData nextDataPoint;
  nextDataPoint.gravity = theInertialData.orientation3D.inverse() * Vector3f(0.f, 0.f, -1.f);
  nextDataPoint.gyro = init ? Vector3a::Zero() : theInertialData.gyro;
  if(theJointAngles.angles[0] != JointAngles::off && theJointAngles.angles[0] != JointAngles::ignore)
  {
    nextDataPoint.measuredAngles = theJointAngles;
    FOREACH_ENUM(Joints::Joint, joint)
      nextDataPoint.measuredAngles.angles[joint] = theJointLimits.limits[joint].limit(nextDataPoint.measuredAngles.angles[joint]);
  }
  if(lastAction.angles[0] != JointAngles::off && lastAction.angles[0] != JointAngles::ignore)
    nextDataPoint.lastActions = lastAction;

  historyBuffer.push_front(nextDataPoint);
}

void WalkingEngine::compile()
{
  // Walk Net
  ASSERT(std::filesystem::exists(modelPath + walkNeuronalNetworkParameters.modelName));

  walkPolicy.compile(Model(modelPath + walkNeuronalNetworkParameters.modelName));
  ASSERT(walkPolicy.valid());

  ASSERT(walkPolicy.numOfInputs() == 1);
  ASSERT(walkPolicy.input(0).rank() == 1);
  ASSERT(walkPolicy.input(0).dims(0) == walkNeuronalNetworkParameters.numInput);

  ASSERT(walkPolicy.numOfOutputs() == 1);
  ASSERT(walkPolicy.output(0).rank() == 1);
  ASSERT(walkPolicy.output(0).dims(0) == walkNeuronalNetworkParameters.numOutput);

  // Unified ball nn
  kickPolicy.compile(Model(modelPath + ballNetworkParameters.kickModel));
  ASSERT(kickPolicy.valid());

  ASSERT(kickPolicy.numOfInputs() == 1);
  ASSERT(kickPolicy.input(0).rank() == 1);
  ASSERT(kickPolicy.input(0).dims(0) == ballNetworkParameters.numInput);

  ASSERT(kickPolicy.numOfOutputs() == 1);
  ASSERT(kickPolicy.output(0).rank() == 1);
  ASSERT(kickPolicy.output(0).dims(0) == walkNeuronalNetworkParameters.numOutput);

  // Old ball nn
  oldKickPolicy.compile(Model(modelPath + oldBallNetworkParameters.kickModel));
  ASSERT(oldKickPolicy.valid());

  ASSERT(oldKickPolicy.numOfInputs() == 1);
  ASSERT(oldKickPolicy.input(0).rank() == 1);
  ASSERT(oldKickPolicy.input(0).dims(0) == oldBallNetworkParameters.numInput);

  ASSERT(oldKickPolicy.numOfOutputs() == 1);
  ASSERT(oldKickPolicy.output(0).rank() == 1);
  ASSERT(oldKickPolicy.output(0).dims(0) == walkNeuronalNetworkParameters.numOutput);
}

void WalkingEngine::update(WalkStepData& walkStepData)
{
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

  walkStepData.yHipOffset = theRobotDimensions.yHipOffset; // Unknown
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
  walkingEngineOutput.maxSpeed = configuredParameters.fastWalkSpeed.maxSpeed;
  walkingEngineOutput.maxSpeedBackwards = configuredParameters.fastWalkSpeed.maxSpeedBackwards;
  walkingEngineOutput.walkStepDuration = 1.f; // unused but set to prevent false usage side effects

  walkingEngineOutput.maxStepSize = configuredParameters.fastWalkSpeed.maxSpeed;
  walkingEngineOutput.maxBackwardStepSize = configuredParameters.fastWalkSpeed.maxSpeedBackwards;

  walkingEngineOutput.maxPossibleStepSize = configuredParameters.maxPossibleSpeed.maxSpeed;
  walkingEngineOutput.maxPossibleBackwardStepSize = configuredParameters.maxPossibleSpeed.maxSpeedBackwards;

  walkingEngineOutput.energyEfficientWalkSpeed = configuredParameters.slowWalkSpeed.maxSpeed;
  ASSERT(configuredParameters.slowWalkSpeed.maxSpeed.rotation < configuredParameters.fastWalkSpeed.maxSpeed.rotation);
  ASSERT(configuredParameters.slowWalkSpeed.maxSpeed.translation.x() < configuredParameters.fastWalkSpeed.maxSpeed.translation.x());
  ASSERT(configuredParameters.slowWalkSpeed.maxSpeed.translation.y() < configuredParameters.fastWalkSpeed.maxSpeed.translation.y());
  ASSERT(configuredParameters.normalWalkSpeed.maxSpeed.rotation < configuredParameters.fastWalkSpeed.maxSpeed.rotation);
  ASSERT(configuredParameters.normalWalkSpeed.maxSpeed.translation.x() < configuredParameters.fastWalkSpeed.maxSpeed.translation.x());
  ASSERT(configuredParameters.normalWalkSpeed.maxSpeed.translation.y() < configuredParameters.fastWalkSpeed.maxSpeed.translation.y());
  walkingEngineOutput.noEfficientWalkSpeed = configuredParameters.normalWalkSpeed.maxSpeed;
}

void WalkingEngine::update(WalkGenerator& walkGenerator)
{
  bool calcVelocity = true;
  if(lastFrameInfo == 0 || lastFrameInfo > theFrameInfo.time)
  {
    lastFrameInfo = theFrameInfo.time;
    calcVelocity = false;
  }
  const float numberFrames = std::max(1.f, std::floor((theFrameInfo.time - lastFrameInfo) / 2.f));
  if(numberFrames > 2.5f || theJointAngles.angles[Joints::lHipPitch] == JointAngles::off || lastMeasurement.angles[Joints::lHipPitch] == JointAngles::off) // Booster robots sometimes have longer data drops. In that case it is better to use boosters velocity value
    calcVelocity = false;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    const Angle oldVel = lastMeasurement.velocity[joint];
    const Angle newVel = !calcVelocity ? static_cast<float>(theJointAngles.velocity[joint]) : (theJointAngles.angles[joint] - lastMeasurement.angles[joint]) * 500.f / numberFrames;
    lastMeasurement.velocity[joint] += Rangef(std::min(-100_deg, -oldVel), std::max(100_deg, -oldVel)).limit(newVel - oldVel);
  }
  lastFrameInfo = theFrameInfo.time;
  lastMeasurement.angles = theJointAngles.angles;

  walkGenerator.createPhase = [this](const Pose2f& step, const MotionPhase& lastPhase, float)->std::unique_ptr<MotionPhase>
  {
    return std::make_unique<WalkPhase>(*this, step, lastPhase);
  };

  walkGenerator.isNextLeftPhase = [this](const MotionPhase& lastPhase, const Pose2f& stepTarget)
  {
    switch(lastPhase.type)
    {
      case MotionPhase::walk:
      {
        const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
        return lastWalkPhase.tBase < 0.44f || lastWalkPhase.tBase > 0.94f;
      }
      case MotionPhase::stand:
      {
        return stepTarget.translation.y() != 0.f // first step based on side translation
               ? stepTarget.translation.y() > 0
               : (stepTarget.rotation != 0_deg ? stepTarget.rotation > 0_deg // else first step based rotation
                  :  theSoleHeightDifference.difference > 0.f); // otherwise based on support foot
      }
      default:
        return theSoleHeightDifference.difference > 0.f;
    }
  };

  walkGenerator.wasLastPhaseLeftPhase = [this](const MotionPhase& lastPhase)
  {
    if(lastPhase.type != MotionPhase::walk)
      return theSoleHeightDifference.difference < 0.f;

    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    return lastWalkPhase.isLeftPhase;
  };

  walkGenerator.isWalkDelayPossible = [](const MotionPhase&, const float, const Pose2f&, const bool)
  {
    return false;
  };

  walkGenerator.getRotationRange = [this](const bool isLeftPhase, const Pose2f& walkSpeedRatio, const Pose2f& walkTarget)
  {
    // TODO rotation currently only used from normal walk speed
    Rangef maxRotationSpeed = Rangef(commonSpeedParameters.rotationTargetScaling.walkSpeed.min, commonSpeedParameters.rotationTargetScaling.walkSpeed.max);

    // Clip based on currently max allowed walk speeds
    maxRotationSpeed.min = std::min(maxRotationSpeed.min, static_cast<float>(configuredParameters.fastWalkSpeed.maxSpeed.rotation));
    maxRotationSpeed.max = std::min(maxRotationSpeed.max, static_cast<float>(configuredParameters.fastWalkSpeed.maxSpeed.rotation));

    // Calc speed ratios
    const float rotationRatio = std::min(std::abs(walkSpeedRatio.rotation), mapToRange(std::abs(walkTarget.rotation), commonSpeedParameters.rotationTargetScaling.walkSpeed.min, commonSpeedParameters.rotationTargetScaling.walkSpeed.max, maxRotationSpeed.min, maxRotationSpeed.max) / configuredParameters.fastWalkSpeed.maxSpeed.rotation);

    const Angle rotation = configuredParameters.fastWalkSpeed.maxSpeed.rotation;
    const float innerTurn = 2.f * 0.5f * rotation * rotationRatio;
    const float outerTurn = 2.f * (1.f - 0.5f) * rotation * std::abs(walkSpeedRatio.rotation);
    return Rangea(isLeftPhase ? -innerTurn : -outerTurn, isLeftPhase ? outerTurn : innerTurn);
  };

  walkGenerator.getStepRotationRangeOther = [this](const bool, const Pose2f& walkSpeedRatio, const Vector2f& step,
                                                   const bool isFastWalk, const std::vector<Vector2f>& translationPolygon,
                                                   const bool ignoreXTranslation, const bool isMaxPossibleStepSize)
  {
    Vector2f walkStep = step;
    ASSERT(translationPolygon.size() >= 4);
    if(translationPolygon.size() < 4)
      return Rangea(0_deg, 0_deg);

    const float defaultMaxSide = configuredParameters.maxPossibleSpeed.maxSpeed.translation.y();

    if(!isMaxPossibleStepSize)
    {
      if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), walkStep))
      {
        Vector2f p1;
        VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f),
                                                               walkStep / walkStep.norm()), p1, false));
        walkStep = p1;
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
      if(walkStep.x() < 0)
        stepRatio.x() = walkStep.x() / xRange.min;
      else if(walkStep.x() > 0.f)
        stepRatio.x() = walkStep.x() / xRange.max;
    }
    if(walkStep.y() < 0)
      stepRatio.y() = walkStep.y() / yRange.min;
    else if(walkStep.y() > 0.f)
      stepRatio.y() = walkStep.y() / yRange.max;

    stepRatio.x() = Rangef::ZeroOneRange().limit(stepRatio.x());
    stepRatio.y() = Rangef::ZeroOneRange().limit(stepRatio.y());

    // Ensure stability
    if(isMaxPossibleStepSize)
    {
      stepRatio.y() = std::sqrt(stepRatio.y());
      stepRatio.x() = std::sqrt(stepRatio.x());
    }

    Range rotationRange = getMaxRotationToStepFactor(isFastWalk, stepRatio);
    rotationRange.min *= std::abs(walkSpeedRatio.rotation);
    rotationRange.max *= std::abs(walkSpeedRatio.rotation);
    return rotationRange;
  };

  walkGenerator.getStepRotationRange = [&walkGenerator](const bool isLeftPhase, const Pose2f& walkSpeedRatio, const Vector2f& step,
                                                        const bool isFastWalk, const MotionPhase& lastPhase, const bool ignoreXTranslation, const bool clipTranslation)
  {
    std::vector<Vector2f> translationPolygon;
    std::vector<Vector2f> translationPolygonNoCenter;
    walkGenerator.getTranslationPolygon(isLeftPhase, 0, lastPhase, walkSpeedRatio, step, translationPolygon, translationPolygonNoCenter, isFastWalk, false, false);

    return walkGenerator.getStepRotationRangeOther(isLeftPhase, walkSpeedRatio, step, isFastWalk, translationPolygon, ignoreXTranslation, clipTranslation);
  };

  walkGenerator.getTranslationPolygon = [this](const bool, float rotation, const MotionPhase& lastPhase, const Pose2f& walkSpeedRatio, const Pose2f& walkTarget, std::vector<Vector2f>& translationPolygon, std::vector<Vector2f>& translationPolygonNoCenter, const bool fastWalk, const bool useMaxPossibleStepSize, const bool useWalkSpeedScaling)
  {
    bool useFastWalk = fastWalk;
    // After an InWalkKick, the next steps are balance steps to ensure that the robot will not fall
    Vector2f forwardBalance = Vector2f(0.f, 0.f);
    Pose2f useWalkSpeedRatio = walkSpeedRatio;

    Vector2f frontLeft, backRight;
    const Pose2f lastStep = lastPhase.type == MotionPhase::walk ? static_cast<const WalkPhase&>(lastPhase).step : Pose2f();
    const Vector2f maxStepSizeChange = commonSpeedParameters.maxAcceleration; // TODO should be a value per second and scaled down to 20 ms time
    const Vector2f maxStepSizeChangeToZero = commonSpeedParameters.maxDeceleration;
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

    backRight.y() = -3000.f;
    frontLeft.y() = 3000.f;

    // Get max walk speed
    Vector2f useSpeed = useMaxPossibleStepSize ? configuredParameters.maxPossibleSpeed.maxSpeed.translation : configuredParameters.fastWalkSpeed.maxSpeed.translation;
    const float useSpeedBackwards = useMaxPossibleStepSize ? configuredParameters.maxPossibleSpeed.maxSpeedBackwards : configuredParameters.fastWalkSpeed.maxSpeedBackwards;

    // Limit to maximum speed (which is influenced by the rotation).
    // When the arms are on the back, the robot will unintentionally turn more in each step.
    // Allow more rotation + translation, so the robot does not walk that much slower
    Vector2f stepSizeFactor = getStepSizeFactor(rotation, useFastWalk, useWalkSpeedRatio.translation);

    // Walk target scaling
    Rangef maxXSpeedForward = Rangef(commonSpeedParameters.xTargetScaling.walkSpeed.min, commonSpeedParameters.xTargetScaling.walkSpeed.max);
    Rangef maxXSpeedBackward = maxXSpeedForward;
    Rangef maxYSpeed = Rangef(commonSpeedParameters.yTargetScaling.walkSpeed.min, commonSpeedParameters.yTargetScaling.walkSpeed.max);

    // Clip based on currently max allowed walk speeds
    maxXSpeedForward.min = std::min(maxXSpeedForward.min, useSpeed.x());
    maxXSpeedForward.max = std::min(maxXSpeedForward.max, useSpeed.x());
    maxXSpeedBackward.min = std::max(maxXSpeedBackward.max, useSpeedBackwards);
    maxXSpeedBackward.max = std::max(maxXSpeedBackward.min, useSpeedBackwards);
    maxYSpeed.min = std::min(maxYSpeed.min, useSpeed.y());
    maxYSpeed.max = std::min(maxYSpeed.max, useSpeed.y());

    // Calc speed ratios
    float normVal = std::max(commonSpeedParameters.xTargetScaling.targetDistance.max, commonSpeedParameters.yTargetScaling.targetDistance.max);
    if(useWalkSpeedScaling)
      normVal = walkTarget.translation.norm();

    const float xForwardRatio = std::min(stepSizeFactor.x(), mapToRange(normVal, commonSpeedParameters.xTargetScaling.targetDistance.min, commonSpeedParameters.xTargetScaling.targetDistance.max, maxXSpeedForward.min, maxXSpeedForward.max) / useSpeed.x());
    const float yForwardRatio = std::min(stepSizeFactor.y(), mapToRange(normVal, commonSpeedParameters.yTargetScaling.targetDistance.min, commonSpeedParameters.yTargetScaling.targetDistance.max, maxYSpeed.min, maxYSpeed.max) / useSpeed.y());
    const float xBackwardRatio = std::min(stepSizeFactor.x(), mapToRange(normVal, commonSpeedParameters.xTargetScaling.targetDistance.min, commonSpeedParameters.xTargetScaling.targetDistance.max, maxXSpeedBackward.min, maxXSpeedBackward.max) / useSpeedBackwards);

    // Calc polygon boundaries
    backRight.x() = std::max(backRight.x(), xBackwardRatio * -useSpeedBackwards);
    backRight.y() = std::max(backRight.y(), yForwardRatio * -useSpeed.y());
    frontLeft.x() = std::min(frontLeft.x(), xForwardRatio * useSpeed.x());
    frontLeft.y() = std::min(frontLeft.y(), yForwardRatio * useSpeed.y());

    // Make sure some minimum exists
    const float useMinXBackwardTranslation = stepSizeParameters.minXBackwardTranslationFastRange.max;
    frontLeft.x() = std::max(stepSizeParameters.minXForwardTranslationFast * useWalkSpeedRatio.translation.x(), frontLeft.x());
    backRight.x() = std::min(useMinXBackwardTranslation * useWalkSpeedRatio.translation.x(), backRight.x());

    // Step size in x translation has a min size
    const float maxMinStepX = std::min(stepSizeParameters.minXTranslationStep, (useWalkSpeedRatio.translation.x() >= 0.f ? useSpeed.x() : useSpeedBackwards) * std::abs(useWalkSpeedRatio.translation.x()) + 0.01f);

    backRight.x() = std::min(backRight.x(), -maxMinStepX);
    frontLeft.x() = std::max(frontLeft.x(), maxMinStepX);

    // (0,0) must be part of the rectangle.
    backRight.x() = std::min(backRight.x(), -.01f);
    frontLeft.x() = std::max(frontLeft.x(), .01f);
    backRight.y() = std::min(backRight.y(), -.01f);
    frontLeft.y() = std::max(frontLeft.y(), .01f);

    Vector2f frontLeftNoCenter = frontLeft;
    Vector2f backRightNoCenter = backRight;

    const float maxSideRatio = Rangef::ZeroOneRange().limit(std::max(0.f, std::abs(theVelocityEstimation.highestLastQuarterSecond.translation.x()) - 500.f) / 1500.f);
    backRight.y() = std::max(backRight.y(), mapToRange(maxSideRatio, 0.f, 1.f, -configuredParameters.maxPossibleSpeed.maxSpeed.translation.y(), -configuredParameters.maxSideAtMaxForward));
    frontLeft.y() = std::min(frontLeft.y(), mapToRange(maxSideRatio, 0.f, 1.f, configuredParameters.maxPossibleSpeed.maxSpeed.translation.y(), configuredParameters.maxSideAtMaxForward));

    generateTranslationPolygon(translationPolygon, backRight, frontLeft, useMaxPossibleStepSize);
    generateTranslationPolygon(translationPolygonNoCenter, backRightNoCenter, frontLeftNoCenter, useMaxPossibleStepSize);

    ASSERT(translationPolygon.size() != 0);
    ASSERT(translationPolygonNoCenter.size() != 0);
  };

  walkGenerator.generateTranslationPolygon = [this](const bool, const Angle rotation, const Pose2f& walkSpeedRatio,
                                                    std::vector<Vector2f>& translationPolygon, Vector2f backRight, Vector2f frontLeft,
                                                    const bool useFastWalk, const bool useMaxPossibleStepSize)
  {
    const Vector2f stepSizeFactor = getStepSizeFactor(rotation, useFastWalk, walkSpeedRatio.translation);

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

  walkGenerator.wasLastPhaseInWalkKick = [](const MotionPhase&)
  {
    return false;
  };
}

Rangea WalkingEngine::getMaxRotationToStepFactor(const bool isFastWalk, const Vector2f& stepRatio)
{
  // Reverses getStepSizeFactor()
  const Vector2a& reduceOffset = isFastWalk ? stepSizeParameters.reduceTranslationFromRotationBaseOffset : stepSizeParameters.reduceTranslationFromRotationBaseFastOffset;
  const Vector2a& reduceThreshold = isFastWalk ? stepSizeParameters.noTranslationFromRotationFastThreshold : stepSizeParameters.noTranslationFromRotationThreshold;

  const Angle maxRotY = std::sqrt(std::max(0.f, -(stepRatio.y() - 1.f))) * std::max(reduceThreshold.y() - reduceOffset.y(), 0.f) + reduceOffset.y();
  const Angle maxRotX = std::sqrt(std::max(0.f, -(stepRatio.x() - 1.f))) * std::max(reduceThreshold.x() - reduceOffset.x(), 0.f) + reduceOffset.x();
  const Angle maxRot = std::max(maxRotX, maxRotY);

  return Rangea(-maxRot, maxRot);
}

Vector2f WalkingEngine::getStepSizeFactor(const Angle rotation, const bool isFastWalk, const Vector2f& walkSpeedRatio)
{
  const Vector2a& reduceOffset = isFastWalk ? stepSizeParameters.reduceTranslationFromRotationBaseOffset : stepSizeParameters.reduceTranslationFromRotationBaseFastOffset;
  const Vector2a& reduceThreshold = isFastWalk ? stepSizeParameters.noTranslationFromRotationFastThreshold : stepSizeParameters.noTranslationFromRotationThreshold;

  const float tFactorX = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - reduceOffset.x()) / (reduceThreshold.x() - reduceOffset.x()))));
  const float tFactorY = std::max(0.f, 1.f - sqr(std::max(0.f, (std::abs(rotation) - reduceOffset.y()) / (reduceThreshold.y() - reduceOffset.y()))));
  return Vector2f(std::min(tFactorX, std::abs(walkSpeedRatio.x())), std::min(tFactorY, std::abs(walkSpeedRatio.y())));
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
  const std::vector<Vector2f>& original = useMaxPossibleStepSize ? translationPolygonMaxPossible : translationPolygon;
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

WalkPhase::WalkPhase(WalkingEngine& engine, Pose2f useStepTarget, const MotionPhase& lastPhase):
  MotionPhase(MotionPhase::walk),
  engine(engine)
{
  step = useStepTarget;
  shouldStop = step == Pose2f();
  slowWalkStart = shouldStop ? engine.theFrameInfo.time - engine.walkNeuronalNetworkParameters.timeLowWalkSpeedForStand : engine.theFrameInfo.time;
  tBase = engine.motionCycleTime;
  frequency = rawFrequency = engine.frequencyParametersWalk.base;

  leftArmInterpolationStart = engine.theFrameInfo.time;
  rightArmInterpolationStart = engine.theFrameInfo.time;
  leftArm = engine.theJointRequest;
  rightArm = engine.theJointRequest;

  oldBall = engine.theMotionRequest.ballEstimate.position;

  lastTarget.angles = engine.theJointRequest.angles;
  lastModelRequest = engine.theFrameInfo.time;
  leftArmInterpolationTime = rightArmInterpolationTime = engine.armParameters.armInterpolationTime;
  const WalkPhase* walkPhase = dynamic_cast<const WalkPhase*>(&lastPhase);
  if((lastPhase.type == MotionPhase::walk || lastPhase.type == MotionPhase::stand) && walkPhase)
  {
    const auto& lastWalkPhase = *walkPhase;
    tBase = lastWalkPhase.tBase;
    lastTarget = lastWalkPhase.lastTarget;
    lastModelRequest = lastWalkPhase.lastModelRequest;
    nextTarget = lastWalkPhase.nextTarget;
    lastWalking = lastWalkPhase.lastWalking;
    lastBallNetBallDistance = lastWalkPhase.lastBallNetBallDistance;
    frequency = lastWalkPhase.frequency;
    rawFrequency = lastWalkPhase.rawFrequency;
    armStartInterpolationTimestamp = lastWalkPhase.armStartInterpolationTimestamp;
    startTarget = lastWalkPhase.startTarget;
    leftArmInterpolationStart = lastWalkPhase.leftArmInterpolationStart;
    rightArmInterpolationStart = lastWalkPhase.rightArmInterpolationStart;
    leftArm = lastWalkPhase.leftArm;
    rightArm = lastWalkPhase.rightArm;
    wasKicking = lastWalkPhase.wasKicking;
    if(lastPhase.type == MotionPhase::stand)
    {
      tBase = engine.theWalkGenerator.isNextLeftPhase(lastPhase, useStepTarget) ? 0.f : 0.5f;
    }
    slowWalkStart = lastWalkPhase.slowWalkStart;
    oldBall = lastWalkPhase.oldBall;
  }
  else
  {
    engine.resetHistoryData();
    lastWalking = engine.theFrameInfo.time > 2000 ? engine.theFrameInfo.time - 2000 : 0;
    armStartInterpolationTimestamp = engine.theFrameInfo.time;
    startTarget = engine.theJointRequest;
    FOREACH_ENUM(Joints::Joint, joint)
    {
      startTarget.angles[joint] = startTarget.angles[joint] == JointAngles::off || startTarget.angles[joint] == JointAngles::ignore || startTarget.stiffnessData.stiffnesses[joint] == 0
                                  ? (engine.theJointAngles.angles[joint] == JointAngles::off ? 0_deg : engine.theJointAngles.angles[joint])
                                  : startTarget.angles[joint];

      leftArm.angles[joint] = startTarget.angles[joint];
      rightArm.angles[joint] = startTarget.angles[joint];
    }
  }

  if(wasKicking ||
     !(std::abs(step.rotation) < engine.walkNeuronalNetworkParameters.slowWalkStepSpeed.rotation &&
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
  lastModelRequest = engine.theFrameInfo.time;
  getNextTargetRequest(nextTarget);

  isLeftPhase = type == MotionPhase::stand || tBase < 0.5f;

  tWalk = 0.f;
  if(type == MotionPhase::walk)
    lastWalking = engine.theFrameInfo.time;
}

void WalkPhase::getNextTargetRequest(JointAngles& target)
{
  engine.updateHistoryData(lastTarget);
  ASSERT(engine.historyBuffer.full());
  const Angle odometryRotation = engine.theOdometryDataPreview.rotation - engine.theMotionRequest.odometryData.rotation;
  //Pose2f odometry = (engine.theOdometryDataPreview.inverse() * engine.theMotionRequest.odometryData);
  //Vector2f ball = odometry * engine.theMotionRequest.ballEstimate.position;
  Vector2f ball;
  ball = engine.theMotionRequest.ballEstimate.position;

  // Shift ball position further outwards relative in the kick direction
  // This forces the policy to handle rolling balls more effective
  // TODO only for K1, as directionOffset from KickInfo is ignored
  if(engine.shiftBallPosition)
  {
    const Angle kickDirection = engine.theMotionRequest.targetDirection - odometryRotation;
    ball.rotate(-kickDirection);
    Vector2f leftSoleInKickDirection = (engine.theTorsoMatrix * engine.theRobotModel.soleLeft).translation.head<2>().rotated(-kickDirection);
    Vector2f rightSoleInKickDirection = (engine.theTorsoMatrix * engine.theRobotModel.soleRight).translation.head<2>().rotated(-kickDirection);
    const float interpolation = mapToRange(std::abs(kickDirection), static_cast<float>(80_deg), static_cast<float>(100_deg), 1.f, 0.f);
    const float interpolationSoleDiff = mapToRange(std::abs(leftSoleInKickDirection.y() - rightSoleInKickDirection.y()), 160.f, 220.f, 0.f, 1.f);
    const float leftShift = mapToRange((ball - leftSoleInKickDirection).y(), engine.shiftBallInterpolationRange.min, engine.shiftBallInterpolationRange.max, 0.f, engine.shiftBallPositionValue);
    const float rightShift = mapToRange((ball - rightSoleInKickDirection).y(), -engine.shiftBallInterpolationRange.max, -engine.shiftBallInterpolationRange.min, -engine.shiftBallPositionValue, 0.f);
    ball.y() += interpolationSoleDiff * interpolation * (leftShift + rightShift);
    ball.rotate(kickDirection);
  }

  if(ball.squaredNorm() > sqr(2000.f))
    ball.normalize(2000.f);

  if((ball - oldBall).squaredNorm() > sqr(200.f))
    oldBall = ball;

  // joint sequence
  const std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();

  JointAngles clippedJointAngles = engine.theJointAngles;
  FOREACH_ENUM(Joints::Joint, joint)
    clippedJointAngles.angles[joint] = engine.theJointLimits.limits[joint].limit(clippedJointAngles.angles[joint]);

  if(!engine.ballNetworkParameters.active || type != MotionPhase::walk || shouldStop || !engine.theMotionRequest.obstacleAvoidance.path.empty() ||
     engine.theMotionRequest.shouldInterceptBall || engine.theMotionRequest.shouldWalkOutOfBallLine ||
     ((engine.theMotionRequest.motion != MotionRequest::dribble && engine.theMotionRequest.motion != MotionRequest::walkToBallAndKick) ||
      type == MotionPhase::stand || engine.theFrameInfo.getTimeSince(engine.theMotionRequest.ballTimeWhenLastSeen) > 1000 || engine.theMotionRequest.ballEstimate.position.norm() > engine.ballDistancePolicySwitch + (lastBallNetBallDistance < engine.ballDistancePolicySwitch ? engine.ballHysteresisPolicySwitch : -engine.ballHysteresisPolicySwitch)))
  {
    wasKicking = false;
    lastBallNetBallDistance = engine.theMotionRequest.ballEstimate.position.norm() + engine.ballHysteresisPolicySwitch;
    float* input = engine.walkPolicy.input(0).data();

    for(std::size_t i = engine.historyBuffer.capacity() - 1; i < engine.historyBuffer.capacity(); i--)
    {
      const auto& data = engine.historyBuffer[i];

      // gravity
      *input++ = data.gravity.x();
      *input++ = data.gravity.y();
      *input++ = data.gravity.z();

      // angular momentum
      *input++ = data.gyro.x();
      *input++ = data.gyro.y();
      *input++ = data.gyro.z();

      // Measurements
      for(Joints::Joint j : jointList)
        *input++ = data.measuredAngles.angles[j] - engine.offset.angles[j];

      // Last Requests
      for(Joints::Joint j : jointList)
        *input++ = data.lastActions.angles[j] - engine.offset.angles[j];

      *input++ = 0.f; // Dummy Ball x
      *input++ = 0.f; // Dummy Ball y
    }

    // walk command
    if(type == MotionPhase::walk)
    {
      *input++ = step.translation.x() / 1000.f;
      *input++ = step.translation.y() / 1000.f;
      *input++ = step.rotation;
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

    // Velocities
    for(Joints::Joint j : jointList)
      *input++ = engine.lastMeasurement.velocity[j] * engine.walkNeuronalNetworkParameters.velocityFactor;

    *input++ = rawFrequency;

    //////////////////////////////////
    // Empty fields for ball policy //
    //////////////////////////////////

    // Empty ball flags
    *input++ = 0.f; // strong kick
    *input++ = 0.f; // over/undershoot
    *input++ = 0.f; // just hit

    // Ball direction as sin and cos
    *input++ = 0;
    *input++ = 0;

    // Ball range
    *input++ = 0;

    // Run network.
    STOPWATCH("module:WalkingEngine:apply")
      engine.walkPolicy.apply();

    // Get next request
    const float* output = engine.walkPolicy.output(0).data();
    for(Joints::Joint j : jointList)
      target.angles[j] = engine.walkNeuronalNetworkParameters.actionClipRange.limit(*output++) + engine.offset.angles[j];

    rawFrequency = engine.frequencyParametersWalk.clipRange.limit(*output++);
    frequency = engine.frequencyParametersWalk.clipRange.limit(rawFrequency) + engine.frequencyParametersWalk.base;
  }
  else
  {
    wasKicking = true;
    Angle directionOffset = 0_deg;

    // Ensure dribble uses the normal kick
    const bool isOldKick = engine.theMotionRequest.kickType == KickInfo::rlKickDynamicOld && engine.oldBallNetworkParameters.active;
    const BallNetworkParameters& ballParams = isOldKick ? engine.oldBallNetworkParameters : engine.ballNetworkParameters;
    KickInfo::KickType kickType = engine.theMotionRequest.motion == MotionRequest::dribble && !isOldKick ? KickInfo::rlKickDynamic : engine.theMotionRequest.kickType;

    switch(kickType)
    {
      case KickInfo::rlKickDynamic:
      case KickInfo::rlKickDynamicOld:
      case KickInfo::rlKickStrong:
      case KickInfo::walkForwardStealBallLeft:
      case KickInfo::walkForwardStealBallRight:
        break;
      default:
        kickType = KickInfo::rlKickDynamic;
    }

    if(kickType == KickInfo::rlKickDynamic || kickType == KickInfo::rlKickStrong)
      directionOffset = engine.theKickInfo[kickType].rotationOffset;

    lastBallNetBallDistance = engine.theMotionRequest.ballEstimate.position.norm() - engine.ballHysteresisPolicySwitch;

    CompiledNN& policy = !isOldKick ? engine.kickPolicy : engine.oldKickPolicy;
    float* input = policy.input(0).data();

    const Vector3f gravity = engine.theInertialData.orientation3D.inverse() * Vector3f(0.f, 0.f, -1.f);

    // gravity
    *input++ = gravity.x();
    *input++ = gravity.y();
    *input++ = gravity.z();

    // angular momentum
    *input++ = engine.theInertialData.gyro.x();
    *input++ = engine.theInertialData.gyro.y();
    *input++ = engine.theInertialData.gyro.z();

    *input++ = ball.x() * 0.001f * ballParams.ballFactor;
    *input++ = ball.y() * 0.001f * ballParams.ballFactor;
    *input++ = Angle::normalize(engine.theMotionRequest.targetDirection + directionOffset - odometryRotation) / Constants::pi;

    // walk cycle
    *input++ = type == MotionPhase::walk ? std::cos(2.f * Constants::pi * tBase) : 0.f;
    *input++ = type == MotionPhase::walk ? std::sin(2.f * Constants::pi * tBase) : 0.f;

    // Measurements
    for(Joints::Joint j : jointList)
      *input++ = clippedJointAngles.angles[j] - engine.offset.angles[j];

    // Velocities
    for(Joints::Joint j : jointList)
      *input++ = engine.lastMeasurement.velocity[j] * engine.walkNeuronalNetworkParameters.velocityFactor;

    // Last Requests
    for(Joints::Joint j : jointList)
      *input++ = lastTarget.angles[j] - engine.offset.angles[j];

    *input++ = rawFrequency;

    // Special Feature Requests
    // 0 -> normal kick, 1 -> strong kick
    *input++ = (engine.forceFastKick && !isOldKick) || kickType == KickInfo::rlKickStrong ? 1.f : 0.f; // Is strong kick?
    // 0 -> overshoot in range, 1 -> undershoot in range
    *input++ = 0.f;
    // 0 -> accurate, 1 -> not so accurate
    *input++ = !isOldKick && (engine.forceFastKick || engine.theMotionRequest.alignPrecisely == KickPrecision::justHitTheBall) ? 1.f : 0.f;

    // Ball velocity (3D)
    Vector2f velocity = engine.theMotionRequest.ballEstimate.velocity;
    if(velocity.squaredNorm() > sqr(2000.f))
      velocity.normalize(2000.f);
    *input++ = velocity.x() * 0.001f * ballParams.ballVelFactor; // x
    *input++ = velocity.y() * 0.001f * ballParams.ballVelFactor; // y
    *input++ = 0; // z

    // Ball direction as sin and cos
    const Angle kickDirection = engine.theMotionRequest.targetDirection + directionOffset - odometryRotation;
    *input++ = std::sin(kickDirection); // sin(direction), 0 for direction 0deg
    *input++ = std::cos(kickDirection); // cos(direction), 1 for direction 0deg

    // Ball range
    float targetBallVelocity = ballParams.kickVelocityRange.max;
    if(kickType != KickInfo::rlKickStrong)
    {
      const float kickLengthPower = KickLengthConverter::kickLengthToPower(kickType, engine.theMotionRequest.kickLength, engine.theKickInfo, engine.theKickLengthPair);
      const float maxKickLength = std::max(1.f, engine.theKickInfo[kickType].range.min + (engine.theKickInfo[kickType].range.max - engine.theKickInfo[kickType].range.min) * kickLengthPower); // 0 range crashes
      targetBallVelocity = ballParams.kickVelocityRange.limit(BallPhysics::velocityForDistance(maxKickLength, engine.theBallSpecification));
    }

    *input++ = targetBallVelocity * 0.001f * ballParams.kickRangeFactor;

    // Last ball Pose
    *input++ = oldBall.x() * 0.001f * ballParams.ballFactor;
    *input++ = oldBall.y() * 0.001f * ballParams.ballFactor;

    // Run network.
    STOPWATCH("module:WalkingEngine:apply")
      policy.apply();

    // Get next request
    const float* output = policy.output(0).data();
    for(Joints::Joint j : jointList)
      target.angles[j] = engine.walkNeuronalNetworkParameters.actionClipRange.limit(*output++) + engine.offset.angles[j];

    // Technically
    // rawFrequency = engine.walkNeuronalNetworkParameters.actionClipRange.limit(*output++)
    // Is the correct code. But at RoboCup and Beijing the "wrong" version below was used.
    rawFrequency = engine.frequencyParametersKick.clipRange.limit(*output++);
    frequency = engine.frequencyParametersKick.clipRange.limit(rawFrequency) + engine.frequencyParametersKick.base;
  }
  oldBall = ball;
}

void WalkPhase::update()
{
  if(lastModelRequest > engine.theFrameInfo.time)
    lastModelRequest = engine.theFrameInfo.time;
  tWalk += engine.motionCycleTime;
  tBase = std::fmod(tBase + engine.motionCycleTime * frequency * (type == MotionPhase::walk ? 1.f : 0.f), 1.f);
}

bool WalkPhase::isDone(const MotionRequest& motionRequest) const
{
  // Only allow switching at 50 Hz, to prevent cases in which after a stand phase another stand phase follows, which would let the robot oscillate at 500 Hz
  return (engine.theFrameInfo.getTimeSince(lastModelRequest) >= 20 && tWalk > engine.motionCycleTime) || motionRequest.motion == MotionRequest::prepare;
}

void WalkPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  calcArmJoints(jointRequest);

  jointRequest.angles[Joints::waistYaw] = 0_deg;

  const std::vector<Joints::Joint> jointList = getBoosterLegJointSequence();
  const float armInterpolation = Rangef::ZeroOneRange().limit(engine.theFrameInfo.getTimeSince(armStartInterpolationTimestamp) / engine.armParameters.armInterpolationTime);
  FOREACH_ENUM(Joints::Joint, j)
  {
    if(j == Joints::headPitch || j == Joints::headYaw)
      continue;
    if(std::find(jointList.begin(), jointList.end(), j) != jointList.end())
    {
      lastTarget.angles[j] = nextTarget.angles[j];
      jointRequest.angles[j] = lastTarget.angles[j];
      jointRequest.stiffnessData.stiffnesses[j] = 100;
    }
    else
      jointRequest.angles[j] = startTarget.angles[j] * (1.f - armInterpolation) + jointRequest.angles[j] * armInterpolation;
  }

  if(type == MotionPhase::stand)
  {
    const float ratio = (std::sin((engine.theFrameInfo.time / 500.f) * Constants::pi2) + 1.f) / 2.f;
    jointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = static_cast<int>(engine.standAnkleStiffness.min * ratio + (1.f - ratio) * engine.standAnkleStiffness.max);
    jointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = static_cast<int>(engine.standAnkleStiffness.min * ratio + (1.f - ratio) * engine.standAnkleStiffness.max);
  }

  motionInfo.isMotionStable = true;
  motionInfo.isWalkPhaseInWalkKick = false;
  motionInfo.speed = type == MotionPhase::walk ? Pose2f(step.rotation, step.translation.x(), step.translation.y()) : Pose2f();

  if(engine.theFrameInfo.getTimeSince(lastWalking) >= 2000)
    odometryOffset = Pose2f(0_deg, 0.f, 0.f);
  else
    odometryOffset = engine.theOdometryDataPreview.odometryChange;
}

void WalkPhase::calcArmJoints(JointRequest& jointRequest)
{
  JointAngles armJoints;
  armJoints.angles[Joints::lShoulderPitch] = engine.armParameters.armShoulderPitch + engine.theRobotModel.soleLeft.translation.x() * engine.armParameters.armShoulderPitchFactor / 1000.f;
  armJoints.angles[Joints::lShoulderRoll] = engine.armParameters.armShoulderRoll + std::max(0.f, engine.theRobotModel.limbs[Limbs::tibiaLeft].translation.y() - engine.theRobotDimensions.yHipOffset) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  armJoints.angles[Joints::lElbowPitch] = engine.armParameters.armElbowYaw;
  armJoints.angles[Joints::lElbowYaw] = -30_deg + engine.theRobotModel.soleLeft.translation.x() * engine.armParameters.armShoulderPitchFactor / 1000.f * 1.3f;
  armJoints.angles[Joints::rShoulderPitch] = engine.armParameters.armShoulderPitch + engine.theRobotModel.soleRight.translation.x() * engine.armParameters.armShoulderPitchFactor / 1000.f;
  armJoints.angles[Joints::rShoulderRoll] = -engine.armParameters.armShoulderRoll + std::min(0.f, engine.theRobotModel.limbs[Limbs::tibiaRight].translation.y() + engine.theRobotDimensions.yHipOffset) * engine.armParameters.armShoulderRollIncreaseFactor / 1000.f;
  armJoints.angles[Joints::rElbowPitch] = engine.armParameters.armElbowYaw;
  armJoints.angles[Joints::rElbowYaw] = 30_deg - engine.theRobotModel.soleRight.translation.x() * engine.armParameters.armShoulderPitchFactor / 1000.f * 1.3f;

  // Override ignore arm joints
  if(jointRequest.angles[Joints::lShoulderPitch] != JointAngles::ignore)
  {
    MotionUtilities::copy(jointRequest, leftArm, Joints::firstLeftArmJoint, Joints::firstRightArmJoint);
    leftArmInterpolationStart = engine.theFrameInfo.time;
    leftArmInterpolationTime = std::max(std::abs(leftArm.angles[Joints::lShoulderPitch] - armJoints.angles[Joints::lShoulderPitch]), std::abs(leftArm.angles[Joints::lShoulderRoll] - armJoints.angles[Joints::lShoulderRoll])) / engine.armParameters.standInterpolationVelocity * 1000.f;
  }
  if(jointRequest.angles[Joints::rShoulderPitch] != JointAngles::ignore)
  {
    MotionUtilities::copy(jointRequest, rightArm, Joints::firstRightArmJoint, Joints::firstNoneArmJoint);
    rightArmInterpolationStart = engine.theFrameInfo.time;
    rightArmInterpolationTime = std::max(std::abs(rightArm.angles[Joints::rShoulderPitch] - armJoints.angles[Joints::rShoulderPitch]), std::abs(rightArm.angles[Joints::rShoulderRoll] - armJoints.angles[Joints::rShoulderRoll])) / engine.armParameters.standInterpolationVelocity * 1000.f;
  }

  // Interpolate left arm
  if(jointRequest.angles[Joints::lShoulderPitch] == JointAngles::ignore && leftArmInterpolationTime > 0)
  {
    for(int joint = Joints::firstLeftArmJoint; joint < Joints::firstRightArmJoint; ++joint)
    {
      const float ratio = std::min(1.f, engine.theFrameInfo.getTimeSince(leftArmInterpolationStart) / leftArmInterpolationTime);
      jointRequest.angles[joint] = leftArm.angles[joint];
      jointRequest.angles[joint] += ratio * (armJoints.angles[joint] - leftArm.angles[joint]);
      jointRequest.stiffnessData.stiffnesses[joint] = StiffnessData::useDefault;
    }
  }
  // Interpolate right arm
  if(jointRequest.angles[Joints::rShoulderPitch] == JointAngles::ignore && rightArmInterpolationTime > 0)
  {
    for(int joint = Joints::firstRightArmJoint; joint < Joints::firstNoneArmJoint; ++joint)
    {
      const float ratio = std::min(1.f, engine.theFrameInfo.getTimeSince(rightArmInterpolationStart) / rightArmInterpolationTime);
      jointRequest.angles[joint] = rightArm.angles[joint];
      jointRequest.angles[joint] += ratio * (armJoints.angles[joint] - rightArm.angles[joint]);
      jointRequest.stiffnessData.stiffnesses[joint] = StiffnessData::useDefault;
    }
  }
}

std::unique_ptr<MotionPhase> WalkPhase::createNextPhase(const MotionPhase& defaultNextPhase) const
{
  if(defaultNextPhase.type == MotionPhase::prepare)
    return std::unique_ptr<MotionPhase>();

  // There must be another walk phase if the robot should transition into something other than walking and the feet are not next to each other.
  if(defaultNextPhase.type != MotionPhase::walk && type != MotionPhase::stand)
    return std::make_unique<WalkPhase>(engine, Pose2f(), *this);
  return std::unique_ptr<MotionPhase>();
}

std::vector<Joints::Joint> WalkPhase::getBoosterLegJointSequence()
{
  if(engine.walkNeuronalNetworkParameters.useWaist)
    return engine.boosterWaistJoints;
  return engine.boosterJoints;
}
