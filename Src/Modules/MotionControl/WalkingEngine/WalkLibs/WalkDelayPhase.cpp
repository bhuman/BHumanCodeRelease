#include "../WalkingEngine.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings3D.h"
#include "Tools/Motion/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"

WalkDelayPhase::WalkDelayPhase(WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase, const float delay, const float height,
                               const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback, const WalkKickStep& walkKickStep) :
  WalkPhase(engine, stepTarget, lastPhase, true, createNextPhaseCallback, walkKickStep),
  delay(std::max(engine.walkDelayParameters.minDelay, std::ceil(delay / engine.motionCycleTime) * engine.motionCycleTime + 0.0001f)), // ensure the full delay is applied
  percentHeight(Rangef::ZeroOneRange().limit(height))
{
  forwardL = forwardL0;
  forwardR = forwardR0;
  sideL = sideL0;
  sideR = sideR0;
  turnRL = turnRL0;
  footHL = footHL0;
  footHR = footHR0;
  originalFootHL0 = footHL0;
  originalFootHR0 = footHR0;
  timeActive += engine.motionCycleTime; // one frame shall have passed
  lastPhaseWasKick = timeWhenLastKick == engine.theFrameInfo.time;

  if(lastPhase.type == MotionPhase::stand)
  {
    const auto& lastWalkPhase = static_cast<const WalkPhase&>(lastPhase);
    if(lastWalkPhase.isDelay)
    {
      const auto& lastDelayPhase = static_cast<const WalkDelayPhase&>(lastPhase);
      timeActive += lastDelayPhase.timeActive;

      forwardL0 = lastDelayPhase.forwardL0;
      forwardR0 = lastDelayPhase.forwardR0;
      sideL0 = lastDelayPhase.sideL0;
      sideR0 = lastDelayPhase.sideR0;
      turnRL0 = lastDelayPhase.turnRL0;
      footHL0 = lastDelayPhase.footHL0;
      footHR0 = lastDelayPhase.footHR0;
      originalFootHL0 = lastDelayPhase.footHL0;
      originalFootHR0 = lastDelayPhase.footHR0;

      forwardL = forwardL0;
      forwardR = forwardR0;
      sideL = sideL0;
      sideR = sideR0;
      turnRL = turnRL0;
      footHL = footHL0;
      footHR = footHR0;
      originalFootHL0 = footHL0;
      originalFootHR0 = footHR0;
    }
  }

  if(!isWalkDelayPossible(forwardL0, forwardR0, sideL0, sideR0, footHL0, footHR0, turnRL0, currentWalkHipHeight,
                          soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR,
                          isLeftPhase, lastPhaseWasKick, delay, engine.walkDelayParameters, engine,
                          walkKickStep.usedWalkDelay))
  {
    OUTPUT_WARNING("WalkDelayPhase: Delay is not possible!");
    ANNOTATION("WalkDelayPhase", "Delay is not possible!");
    this->delay = 0.f;
    finishedDelay = true;
  }

  update();
}

void WalkDelayPhase::update()
{
  // one frame has passed
  timeActive += engine.motionCycleTime;
  if(walkKickStep.currentKick != WalkKicks::none && engine.configuredParameters.dynamicKickDelayUpdate)
  {
    if(timeActive < delay) // Update delay for kicks
      delay = updatedDelayForKick();
  }

  if(timeActive >= delay)
  {
    if(!finishedDelay)
    {
      finishedDelay = true;
      // Make sure joint play controller knows the actual current values
      jointPlayOffsetController = std::make_unique<JointPlayOffsetController>(engine.theJointRequest);
      if(walkKickStep.usedWalkDelay)
      {
        const float sideShift = getSideShift(lastPhaseWasKick, delay, engine.walkDelayParameters) * (isLeftPhase ? 1.f : -1.f);
        engine.theWalkKickGenerator.dynamicWalkKickStepUpdate(walkKickStep, 2.f * engine.motionCycleTime);
        for(std::size_t i = 0; i < walkKickStep.keyframe.size(); i++)
        {
          walkKickStep.keyframe[i].stepTargetConverted.translation.y() -= sideShift * 2.f;
          walkKickStep.keyframe[i].stepTargetSwingConverted.translation.y() += sideShift * 2.f;
          walkKickStep.keyframe[i].stepTarget.translation.y() -= sideShift * 2.f;
        }
      }
    }
    else
    {
      WalkPhase::update();
      return;
    }
  }
}

float WalkDelayPhase::updatedDelayForKick()
{
  // TODO Calculation is wrong? Use kick foot as scsCognition, not the zero step
  // TODO check if kick just moves kick foot forward
  // Apply scsCognition transformation
  if(engine.theMotionRequest.ballEstimate.velocity == Vector2f::Zero())
    return delay;

  if(engine.theMotionRequest.ballEstimateTimestamp != lastBallTimestamp)
  {
    lastBallTimestamp = engine.theMotionRequest.ballEstimateTimestamp;
    ballTimeOffset = 0.f;
  }
  else
    ballTimeOffset += engine.motionCycleTime;

  Pose2f scsCognition = Motion::Transformation::getTransformationToZeroStep(engine.theTorsoMatrix, engine.theRobotModel, engine.theRobotDimensions.yHipOffset + engine.kinematicParameters.yHipOffset, engine.theMotionRequest.odometryData, engine.theOdometryDataPreview, isLeftPhase);
  scsCognition.rotation -= walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.rotation;
  const Geometry::Line ballLine = Geometry::Line(scsCognition * engine.theMotionRequest.ballEstimate.position, engine.theMotionRequest.ballEstimate.velocity.rotated(scsCognition.rotation));
  Vector2f hitPoint;
  const float hipOffset = (isLeftPhase ? 1.f : -1.f) * (engine.theRobotDimensions.yHipOffset + engine.kinematicParameters.yHipOffset);

  const Vector2f kickSoleToe(engine.theRobotDimensions.soleToFrontEdgeLength, hipOffset);
  // TODO use current yOffset of kick sole
  if(Geometry::getIntersectionOfLines(ballLine, Geometry::Line(kickSoleToe, Vector2f(1.f, 0.f)), hitPoint))
  {
    COMPLEX_DRAWING3D("module:WalkDelayPhase:hitPoint")
    {
      Vector2f hitPointInSole = (scsCognition.inverse() * hitPoint);
      CROSS3D("module:WalkDelayPhase:hitPoint", hitPointInSole.x(), hitPointInSole.y(), (isLeftPhase ? engine.theRobotModel.soleLeft : engine.theRobotModel.soleRight).translation.z(), 10.f, 10.f, ColorRGBA::black);
    }
    if(!Approx::isZero((hitPoint - ballLine.base).angle() - engine.theMotionRequest.ballEstimate.velocity.rotated(scsCognition.rotation).angle(), 0.1f))
      return timeActive;
    const float timeForBall = BallPhysics::timeForDistance(engine.theMotionRequest.ballEstimate.velocity, (hitPoint - ballLine.base).norm(), engine.theBallSpecification.friction) - ballTimeOffset;
    if(timeForBall == std::numeric_limits<float>::max())
      return delay;

    walkKickStep.currentKickVariant->ballEstimationTime = timeForBall;

    return timeForBall + timeActive - mapToRange(hitPoint.x() - engine.theRobotDimensions.soleToFrontEdgeLength, engine.walkDelayParameters.kickTimeBallDistanceScaling.min, engine.walkDelayParameters.kickTimeBallDistanceScaling.max, engine.walkDelayParameters.kickTimeOffset.min, engine.walkDelayParameters.kickTimeOffset.max);
  }
  else
    return delay;
}

float WalkDelayPhase::getSideShift(const bool lastPhaseWasKick, const float delay, const WalkDelayParameters& params)
{
  return lastPhaseWasKick ? params.sideShift.max : mapToRange(delay, params.sideShiftDelayInterpolation.min, params.sideShiftDelayInterpolation.max, params.sideShift.min, params.sideShift.max);
}

void WalkDelayPhase::calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  // delay is over, do calcJoints for normal walk phase
  if(timeActive >= delay)
  {
    WalkPhase::calcJoints(motionRequest, jointRequest, odometryOffset, motionInfo);
    return;
  }

  // calculate some ratios and values
  // idea: the longer the delay -> the higher the swing foot stretch (in the z-axis translation)
  // do some of the height over the complete delay
  // do a big chunk at the very end, to do an impulse
  // for higher delays, also add a side shift for the support foot
  const float heightChange = mapToRange(delay, engine.walkDelayParameters.delayInterpolation.min, engine.walkDelayParameters.delayInterpolation.max, engine.walkDelayParameters.heightOffset.min, engine.walkDelayParameters.heightOffset.max);
  const float ratioBase = delay <= engine.walkDelayParameters.minDelay ? 0.f : mapToRange(timeActive / delay, 0.f, 1.f, 0.f, 1.f);
  const float ratioEnd = mapToRange((timeActive - delay + engine.walkDelayParameters.minDelay) / engine.walkDelayParameters.minDelay, 0.f, 1.f, 0.f, 1.f);
  const float heightShift = (ratioEnd * engine.walkDelayParameters.endHeightShift + ratioBase * (heightChange - engine.walkDelayParameters.endHeightShift)) * percentHeight;
  const float sideShift = getSideShift(lastPhaseWasKick, delay, engine.walkDelayParameters);

  // apply offsets
  if(isLeftPhase)
  {
    footHL0 = originalFootHL0 - heightShift;
    sideR0 += sideShift * engine.motionCycleTime / delay;
    if(Global::getSettings().robotType != Settings::nao || walkKickStep.usedWalkDelay)
      sideL0 += sideShift * engine.motionCycleTime / delay;
  }
  else
  {
    footHR0 = originalFootHR0 - heightShift;
    sideL0 -= sideShift * engine.motionCycleTime / delay;
    if(Global::getSettings().robotType != Settings::nao || walkKickStep.usedWalkDelay)
      sideR0 -= sideShift * engine.motionCycleTime / delay;
  }

  // calculate joint request
  Pose3f leftFootForArms;
  Pose3f rightFootForArms;

  if(walkKickStep.currentKick != WalkKicks::none && engine.configuredParameters.dynamicKickDelayUpdate)
  {
    forwardL -= 0.5f;
    forwardR -= 0.5f;
    forwardL0 -= 0.5f;
    forwardR0 -= 0.5f;
  }

  engine.calcFeetPoses(forwardL, forwardR, sideL0, sideR0, footHL0, footHR0, turnRL, currentWalkHipHeight, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms); // current request

  VERIFY(InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions, 10.f));

  // update regulator
  jointSpeedController->reset(jointRequest.angles, turnRL0);

  // apply gyro balancing
  static_cast<void>(addGyroBalance(jointRequest, JointAngles()));
}

bool WalkDelayPhase::isWalkDelayPossible(float forwardL0, float forwardR0, float sideL0, float sideR0,
                                         float footHL0, float footHR0, const float turnRL0, const float currentWalkHipHeight,
                                         const Angle soleRotationYL, const Angle soleRotationXL,
                                         const Angle soleRotationYR, const Angle soleRotationXR,
                                         const bool isLeftPhase, const bool lastPhaseWasKick, const float delay,
                                         const WalkDelayParameters& params, const WalkingEngine& engine,
                                         const bool usedWalkDelay)
{
  const float sideShift = getSideShift(lastPhaseWasKick, delay, params);

  const float heightChange = mapToRange(delay, engine.walkDelayParameters.delayInterpolation.min, engine.walkDelayParameters.delayInterpolation.max, engine.walkDelayParameters.heightOffset.min, engine.walkDelayParameters.heightOffset.max);
  const float ratioBase = delay <= engine.walkDelayParameters.minDelay ? 0.f : mapToRange(delay / (delay - engine.walkDelayParameters.minDelay), 0.f, 1.f, 0.f, 1.f);
  const float ratioEnd = mapToRange((delay - delay + engine.walkDelayParameters.minDelay) / engine.walkDelayParameters.minDelay, 0.f, 1.f, 0.f, 1.f);
  const float heightShift = (ratioEnd * engine.walkDelayParameters.endHeightShift + ratioBase * (heightChange - engine.walkDelayParameters.endHeightShift)) * 1.f;

  if(isLeftPhase)
  {
    footHL0 -= heightShift;
    sideR0 += sideShift;
    if(usedWalkDelay)
      sideL0 += sideShift;
  }
  else
  {
    footHR0 -= heightShift;
    sideL0 -= sideShift;
    if(usedWalkDelay)
      sideR0 -= sideShift;
  }

  forwardL0 += (forwardL0 > 0.f ? params.translationBuffer : -params.translationBuffer);
  forwardR0 += (forwardR0 > 0.f ? params.translationBuffer : -params.translationBuffer);
  sideL0 += params.translationBuffer;
  sideR0 -= params.translationBuffer;

  // calculate joint request
  Pose3f leftFootForArms;
  Pose3f rightFootForArms;

  engine.calcFeetPoses(forwardL0, forwardR0, sideL0, sideR0, footHL0, footHR0, turnRL0, currentWalkHipHeight,
                       soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms); // current request

  JointRequest request;
  return InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f::Zero(), request, engine.theRobotDimensions, 10.f);
}
