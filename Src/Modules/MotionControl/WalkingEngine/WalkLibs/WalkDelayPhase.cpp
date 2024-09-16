#include "../WalkingEngine.h"
#include "Debugging/Annotation.h"
#include "Tools/Motion/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"

WalkDelayPhase::WalkDelayPhase(WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase, const float delay, const float height,
                               const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback, const WalkKickStep& walkKickStep) :
  WalkPhase(engine, stepTarget, lastPhase, createNextPhaseCallback, walkKickStep),
  delay(std::max(engine.walkDelayParameters.minDelay, std::ceil(delay / Constants::motionCycleTime) * Constants::motionCycleTime + 0.0001f)), // ensure the full delay is applied
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
  timeActive += Constants::motionCycleTime; // one frame shall have passed
  lastPhaseWasKick = timeWhenLastKick == engine.theFrameInfo.time;

  if(!isWalkDelayPossible(forwardL0, forwardR0, sideL0, sideR0, footHL0, footHR0, turnRL0,
                          soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR,
                          isLeftPhase, lastPhaseWasKick, delay, engine.walkDelayParameters, engine,
                          walkKickStep.usedWalkDelay, torsoShift))
  {
    OUTPUT_WARNING("WalkDelayPhase: Delay is not possible!");
    ANNOTATION("WalkDelayPhase", "Delay is not possible!");
    this->delay = 0.f;
  }
}

void WalkDelayPhase::update()
{
  // one frame has passed
  timeActive += Constants::motionCycleTime;
  // delay is over, do update for normal walk phase
  if(timeActive >= delay)
  {
    if(!finishedDelay)
    {
      finishedDelay = true;
      if(walkKickStep.usedWalkDelay)
      {
        const float sideShift = getSideShift(lastPhaseWasKick, delay, engine.walkDelayParameters) * (isLeftPhase ? 1.f : -1.f);
        for(std::size_t i = 0; i < walkKickStep.keyframe.size(); i++)
        {
          walkKickStep.keyframe[i].stepTargetConverted.translation.y() -= sideShift * 2.f;
          walkKickStep.keyframe[i].stepTargetSwingConverted.translation.y() += sideShift * 2.f;
          walkKickStep.keyframe[i].stepTarget.translation.y() -= sideShift * 2.f;
        }
      }
    }
    WalkPhase::update();
  }

  // Update delay for kicks
  // TODO: deactivate for now. Needs to be tested for side effects
  // if(walkKickStep.currentKick != WalkKicks::none)
  //  delay = updatedDelayForKick();
}

float WalkDelayPhase::updatedDelayForKick()
{
  // Apply scsCognition transformation
  const Pose2f scsCognition = Motion::Transformation::getTransformationToZeroStep(engine.theTorsoMatrix, engine.theRobotModel, engine.theRobotDimensions, engine.theMotionRequest.odometryData, engine.theOdometryDataPreview, isLeftPhase);
  const Geometry::Line ballLine = Geometry::Line(scsCognition * engine.theMotionRequest.ballEstimate.position, engine.theMotionRequest.ballEstimate.velocity.rotated(scsCognition.rotation));
  const float sideShift = getSideShift(lastPhaseWasKick, delay, engine.walkDelayParameters) * (isLeftPhase ? 1.f : -1.f);
  const Vector2f stepSupport = walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetConverted.translation - Vector2f(0.f, sideShift) * 2.f;
  const Vector2f stepSwing = walkKickStep.keyframe[walkKickStep.keyframe.size() - 1].stepTargetSwingConverted.translation + Vector2f(0.f, sideShift) * 2.f;
  Vector2f hitPoint;
  const float hipOffset = (isLeftPhase ? 1.f : -1.f) * engine.theRobotDimensions.yHipOffset;
  if(Geometry::getIntersectionOfLines(ballLine, Geometry::Line(Vector2f(0.f, hipOffset), (stepSupport + stepSwing) * 0.5f), hitPoint))
  {
    const float timeForDistanceHitPoint = timeActive + BallPhysics::timeForDistance(engine.theMotionRequest.ballEstimate.velocity, (hitPoint - ballLine.base).norm(), engine.theBallSpecification.friction);
    return timeForDistanceHitPoint - engine.walkDelayParameters.kickTimeOffset;
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
  // idea: the longer the delay -> the higher the swing foot strecht (in the z-axis translation)
  // do some of the height over the complete delay
  // do a big chunk at the very end, to do an impulse
  // for higher delays, also add a side shift for the support foot
  const float heightChange = mapToRange(delay, engine.walkDelayParameters.delayInterpolation.min, engine.walkDelayParameters.delayInterpolation.max, engine.walkDelayParameters.heightOffset.min, engine.walkDelayParameters.heightOffset.max);
  const float ratioBase = delay <= engine.walkDelayParameters.minDelay ? 0.f : mapToRange(timeActive / (delay - engine.walkDelayParameters.minDelay), 0.f, 1.f, 0.f, 1.f);
  const float ratioEnd = mapToRange((timeActive - delay + engine.walkDelayParameters.minDelay) / engine.walkDelayParameters.minDelay, 0.f, 1.f, 0.f, 1.f);
  const float heightShift = (ratioEnd * engine.walkDelayParameters.endHeightShift + ratioBase * (heightChange - engine.walkDelayParameters.endHeightShift)) * percentHeight;
  const float sideShift = getSideShift(lastPhaseWasKick, delay, engine.walkDelayParameters);

  // apply offsets
  if(isLeftPhase)
  {
    footHL0 = originalFootHL0 - heightShift;
    sideR0 += sideShift * Constants::motionCycleTime / delay;
    if(walkKickStep.usedWalkDelay)
      sideL0 += sideShift * Constants::motionCycleTime / delay;
  }
  else
  {
    footHR0 = originalFootHR0 - heightShift;
    sideL0 -= sideShift * Constants::motionCycleTime / delay;
    if(walkKickStep.usedWalkDelay)
      sideR0 -= sideShift * Constants::motionCycleTime / delay;
  }

  // calculate joint request
  Pose3f leftFootForArms;
  Pose3f rightFootForArms;

  engine.calcFeetPoses(forwardL, forwardR, sideL0, sideR0, footHL0, footHR0, turnRL, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms, torsoShift); // current request

  VERIFY(InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions));

  // update regulator
  jointSpeedRegulator->reset(jointRequest.angles, turnRL0);

  // apply gyro balancing
  Angle& ankleLeft = jointRequest.angles[Joints::lAnklePitch];
  Angle& ankleRight = jointRequest.angles[Joints::rAnklePitch];
  currentMaxAnklePitch[Legs::left] = std::max(currentMaxAnklePitch[Legs::left],
                                              std::max(engine.theJointAngles.angles[Joints::lAnklePitch], ankleLeft));
  currentMaxAnklePitch[Legs::right] = std::max(currentMaxAnklePitch[Legs::right],
                                               std::max(engine.theJointAngles.angles[Joints::rAnklePitch], ankleRight));

  const JointAngles appliedGyroChanges = addGyroBalance(jointRequest, JointAngles());
  if(walkState == standing || !isLeftPhase)
  {
    ankleLeft = std::min(ankleLeft, currentMaxAnklePitch[Legs::left]);
    currentMaxAnklePitch[Legs::left] = ankleLeft;
  }
  if(walkState == standing || isLeftPhase)
  {
    ankleRight = std::min(ankleRight, currentMaxAnklePitch[Legs::right]);
    currentMaxAnklePitch[Legs::right] = ankleRight;
  }
}

bool WalkDelayPhase::isWalkDelayPossible(float forwardL0, float forwardR0, float sideL0, float sideR0,
                                         float footHL0, float footHR0, const float turnRL0,
                                         const Angle soleRotationYL, const Angle soleRotationXL,
                                         const Angle soleRotationYR, const Angle soleRotationXR,
                                         const bool isLeftPhase, const bool lastPhaseWasKick, const float delay,
                                         const WalkDelayParameters& params, const WalkingEngine& engine,
                                         const bool usedWalkDelay, std::optional<float> torsoShift)
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

  engine.calcFeetPoses(forwardL0, forwardR0, sideL0, sideR0, footHL0, footHR0, turnRL0,
                       soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms, torsoShift); // current request

  JointRequest request;
  return InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f::Zero(), request, engine.theRobotDimensions);
}
