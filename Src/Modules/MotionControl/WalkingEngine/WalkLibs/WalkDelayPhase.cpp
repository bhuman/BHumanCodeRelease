#include "../WalkingEngine.h"
#include "Debugging/Annotation.h"

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
                          isLeftPhase, lastPhaseWasKick, delay, engine.walkDelayParameters, engine))
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
    WalkPhase::update();
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
  const float heighShift = (ratioEnd * engine.walkDelayParameters.endHeightShift + ratioBase * (heightChange - engine.walkDelayParameters.endHeightShift)) * percentHeight;
  const float sideShift = getSideShift(lastPhaseWasKick, delay, engine.walkDelayParameters);

  // apply offsets
  if(isLeftPhase)
  {
    footHL0 = originalFootHL0 - heighShift;
    sideR0 += sideShift * Constants::motionCycleTime / delay;
  }
  else
  {
    footHR0 = originalFootHR0 - heighShift;
    sideL0 -= sideShift * Constants::motionCycleTime / delay;
  }

  // calculate joint request
  Pose3f leftFootForArms;
  Pose3f rightFootForArms;

  engine.calcFeetPoses(forwardL, forwardR, sideL0, sideR0, footHL0, footHR0, turnRL, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms); // current request

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
                                         const WalkDelayParameters& params, const WalkingEngine& engine)
{
  const float sideShift = getSideShift(lastPhaseWasKick, delay, params);

  if(isLeftPhase)
  {
    footHL0 -= params.heightOffset.max;
    sideR0 += sideShift;
  }
  else
  {
    footHR0 -= params.heightOffset.max;
    sideL0 -= sideShift;
  }

  const float leftOffset = isLeftPhase ? params.translationBuffer : 0.f;
  const float rightOffset = !isLeftPhase ? params.translationBuffer : 0.f;

  forwardL0 += (forwardL0 > 0.f ? leftOffset : -leftOffset);
  forwardR0 += (forwardR0 > 0.f ? rightOffset : -rightOffset);
  sideL0 += leftOffset;
  sideR0 += rightOffset;

  // calculate joint request
  Pose3f leftFootForArms;
  Pose3f rightFootForArms;

  engine.calcFeetPoses(forwardL0, forwardR0, sideL0 + rightOffset, sideR0 + rightOffset, footHL0, footHR0, turnRL0,
                       soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms); // current request

  JointRequest request;
  return InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f::Zero(), request, engine.theRobotDimensions);
}
