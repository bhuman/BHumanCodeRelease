/**
 * @file WalkKickEngine.cpp
 *
 * This file implements a module that provides a walk kick generator.
 *
 * The general procedure is as follows:
 * 1. canStart is executed, to check if the requested kick is possible.
 * This is based on thresholds for the ball position and the executed step.
 *
 * 2. createPhase is executed, to create a walk phase with kick parameters.
 * Here the relative ball positions from the configuration are converted in explicit walk step sizes as a list of keyframes.
 * Afterwards some clippings are applied and follow up calculations like
 * interpolations for the forward/turn kick, execution durations for the keyframes, etc.
 *
 * 3. if the step was a pre step, createPhaseWithNextPhase is executed again.
 *
 * @author Philip Reichenberg
 */

#include "WalkKickEngine.h"
#include "Platform/SystemCall.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/Rotation.h"
#include "Streaming/TypeRegistry.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/KickLengthConverter.h"
#include <regex>
#include <cmath>

MAKE_MODULE(WalkKickEngine);

WalkKickEngine::WalkKickEngine()
{
  // Directly text-to-speech-synthesizing the enum names is hard to understand.
  // Therefore add a space before every camel case.
  FOREACH_ENUM(WalkKicks::Type, t)
    kickTypesSpeech[t] = std::regex_replace(TypeRegistry::getEnumName(t), std::regex("([A-Z])"), " $1");

  // forward and turnOut must have the same number of keyframes
  ASSERT(walkKicksList[WalkKicks::forward].kickKeyFrame.size() == walkKicksList[WalkKicks::turnOut].kickKeyFrame.size());
  for(size_t i = 0; i < walkKicksList[WalkKicks::forward].kickKeyFrame.size(); i++)
    ASSERT(walkKicksList[WalkKicks::forward].kickKeyFrame[i].keyframes.size() == walkKicksList[WalkKicks::turnOut].kickKeyFrame[i].keyframes.size());
  ASSERT(walkKicksList[WalkKicks::forwardSteal].kickKeyFrame.size() == 1);
}

void WalkKickEngine::update(WalkKickGenerator& walkKickGenerator)
{
  walkKickGenerator.drawStep = [this](const Pose2f& step)
  {
    draw(step);
  };

  walkKickGenerator.getVShapeWalkStep = [this](const bool isLeftPhase, const Angle direction)
  {
    return getVShapeWalkStep(isLeftPhase, direction);
  };

  walkKickGenerator.canStart = [this](WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const Rangea& precisionRange,
                                      const bool preStepAllowed, const bool turnKickAllowed)
  {
    // No InWalkKicks after standing
    if(lastPhase.type != MotionPhase::walk)
      return false;

    // After an InWalkKick at least one normal walk phase must follow
    if(theWalkGenerator.wasLastPhaseInWalkKick(lastPhase))
      return false;

    // Diagonal kicks are allowed when we just wanto to hit the ball
    walkKickVariant.diagonalKickState = allowDiagonalKicks && walkKickVariant.precision == KickPrecision::justHitTheBall && walkKickVariant.walkKickType != WalkKicks::forwardSteal ? WalkKickVariant::DiagonalKickState::allowed : WalkKickVariant::DiagonalKickState::none;

    // Get last step size
    Pose2f leftPose(0_deg, 0.f, 0.f);
    Pose2f rightPose(0_deg, 0.f, 0.f);
    Pose2f lastExecutedStep(0_deg, 0.f, 0.f);
    std::tie(leftPose, rightPose, lastExecutedStep) = theWalkGenerator.getStartOffsetOfNextWalkPhase(lastPhase);
    const Pose2f lastStepChange = theWalkGenerator.getLastStepChange(lastPhase);
    const Pose2f maxOfLastStepExecution(std::max(std::abs(lastExecutedStep.rotation), std::abs(lastStepChange.rotation)), std::max(std::abs(lastExecutedStep.translation.x()), std::abs(lastStepChange.translation.x())), std::max(std::abs(lastExecutedStep.translation.y()), std::abs(lastStepChange.translation.y())));
    bool criticalLastStepYTranslation = maxOfLastStepExecution.translation.y() > restrictedYTranslationOfPreStep.y();
    if(walkKickVariant.walkKickType == WalkKicks::forward)
    {
      const Rangef forwardRange(walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : 0_deg, walkKickVariant.kickLeg != Legs::left ? -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset : 0_deg);
      const Angle confAngle = forwardRange.limit(walkKickVariant.direction);
      const float interpolation = Rangef::ZeroOneRange().limit(confAngle / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset) / 0.5f);
      criticalLastStepYTranslation |= maxOfLastStepExecution.translation.x() > turnKickPreviousMaxXStepSize * (1.f - interpolation) + restrictedYTranslationOfPreStep.x() * interpolation;
    }
    else if(walkKickVariant.walkKickType == WalkKicks::turnOut)
      criticalLastStepYTranslation |= maxOfLastStepExecution.translation.x() > restrictedYTranslationOfPreStep.x();

    if(walkKickVariant.precision == KickPrecision::precise && walkKickVariant.walkKickType == WalkKicks::turnOut) // force the robot to stop before the kick
      criticalLastStepYTranslation |= maxOfLastStepExecution.translation.x() > forwardTurnPreciseMaxStepSize;

    // Is left the swing foot?
    bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());

    // Get positon of kicking foot, as if the feet would return first back to zero
    const Pose3f nonKickFoot = walkKickVariant.kickLeg != Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
    Pose3f kickFootPosition;
    if(walkKickVariant.walkKickType == WalkKicks::forwardSteal)
      kickFootPosition = !isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight;
    else
      kickFootPosition = walkKickVariant.kickLeg == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;

    Angle rotationOffset = 0_deg;
    if(walkKickVariant.walkKickType == WalkKicks::forwardSteal)
      rotationOffset = !isLeftPhase ? -forwardStealVFeetAngle : forwardStealVFeetAngle;

    // Calculate ball model relative to the kicking foot
    const Pose3f supportInTorso3D = (theTorsoMatrix * kickFootPosition).rotateZ(rotationOffset);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f scsCognition = supportInTorso.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
    const Vector2f ballModel = scsCognition * BallPhysics::propagateBallPosition(theMotionRequest.ballEstimate.position, theMotionRequest.ballEstimate.velocity, walkKickVariant.ballEstimationTime, theBallSpecification.friction);
    const Rangea precisionRangeWithOdometry(std::min(precisionRange.min - scsCognition.rotation, 0.f), std::max(0.f, precisionRange.max - scsCognition.rotation));

    // Can the kick be executed based on the swing foot and the kick type?
    const bool preStepIsNext = walkKickVariant.kickLeg != (isLeftPhase ? Legs::left : Legs::right);
    bool canKick = (preStepIsNext && preStepAllowed && !(walkKickVariant.walkKickType == WalkKicks::forwardVeryLong || (walkKickVariant.walkKickType == WalkKicks::forwardLong && walkKickVariant.precision == KickPrecision::precise))) ||
                   (!preStepIsNext && walkKickVariant.walkKickType == WalkKicks::forwardVeryLong);
    if(!preStepIsNext && ((walkKickVariant.walkKickType == WalkKicks::forward && std::abs(walkKickVariant.direction) < forwardPreStepSkipMaxKickAngle) || walkKickVariant.walkKickType == WalkKicks::forwardLong || walkKickVariant.walkKickType == WalkKicks::forwardAlternative))
      canKick = true;
    if(walkKickVariant.walkKickType == WalkKicks::sidewardsOuter || walkKickVariant.walkKickType == WalkKicks::forwardSteal)
      canKick = !preStepIsNext && maxOfLastStepExecution.translation.x() < sidewardsOuterForwardRestriction;

    // was last step size too big?
    // this check is needed to ensure that the ball is hit
    if(criticalLastStepYTranslation && walkKickVariant.precision != KickPrecision::justHitTheBall)
      for(WalkKicks::Type kickType : walkKicksWithRestrictedStart)
        if(kickType == walkKickVariant.walkKickType)
          canKick = false;

    WalkKickVariant kick = walkKickVariant;
    bool canKickAfterCheck = canKick && findBestKickExecution(canKick, precisionRangeWithOdometry, ballModel, kick,
                                                              lastStepChange, walkKickVariant.precision, turnKickAllowed, preStepIsNext);
    if(canKickAfterCheck)
      canKickAfterCheck &= canKickStepSize(kick, isLeftPhase, lastPhase);

    if(!canKickAfterCheck && kick.diagonalKickState == WalkKickVariant::allowed)
    {
      kick = walkKickVariant;
      canKick = findBestKickExecutionDiagonal(precisionRangeWithOdometry, kick,
                                              lastStepChange, walkKickVariant.precision, turnKickAllowed, preStepIsNext);
      if(canKick)
        canKick &= canKickStepSize(kick, isLeftPhase, lastPhase);
      if(canKick && kick.diagonalKickState == WalkKickVariant::set)
      {
        OUTPUT_TEXT("diagonal: " << TypeRegistry::getEnumName(walkKickVariant.kickType) << " " << walkKickVariant.direction.toDegrees());
      }
    }
    else
      canKick &= canKickAfterCheck;

    if(walkKickVariant.walkKickType == WalkKicks::forwardSteal)
      canKick &= std::abs((theTorsoMatrix * theRobotModel.soleLeft).rotation.getZAngle() - (theTorsoMatrix * theRobotModel.soleRight).rotation.getZAngle()) > forwardStealVFeetAngle * forwardStealVFeetAngleFactor;

    if(canKick)
    {
      walkKickVariant = kick;
      return true;
    }
    return false;
  };

  walkKickGenerator.createPhase = [this](const WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const bool playSound)
  {
    odometryAtStart = theOdometryDataPreview; // use theOdometryData and not the updated one, because the rotation of the current frame is not added on the kick direction yet
    currentKick = walkKickVariant;

    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());

    // Can the resulting step target be executed?
    kickIndex = (currentKick.walkKickType == WalkKicks::forward || currentKick.walkKickType == WalkKicks::forwardLong || currentKick.walkKickType == WalkKicks::forwardAlternative) &&
                ((currentKick.kickLeg == Legs::left && isLeftPhase) || (currentKick.kickLeg == Legs::right && !isLeftPhase)) ? 1 : 0;

    // TODO the kick can still be aborted in the pre step for some reason... I think. This results in a false sound
    if(playKickSounds && playSound)
      SystemCall::say((kickTypesSpeech[currentKick.walkKickType] + (currentKick.kickLeg == Legs::left ? " left" : " right")).c_str());

    return createPhaseWithNextPhase(lastPhase);
  };

  MODIFY("module:WalkKickEngine:enableDrawings", enableDrawings);
  DECLARE_DEBUG_DRAWING3D("module:WalkKickEngine:nextSupport", "robot");
  draw(theWalkStepData.stepTarget);
}

bool WalkKickEngine::canKickStepSize(WalkKickVariant& kick, const bool isLeftPhase, const MotionPhase& lastPhase)
{
  kick.power = KickLengthConverter::kickLengthToPower(kick.kickType, kick.length, kick.direction, theKickInfo);
  // Can the resulting step target be executed?
  const int kickIndex = (kick.walkKickType == WalkKicks::forward || kick.walkKickType == WalkKicks::forwardLong || kick.walkKickType == WalkKicks::forwardAlternative) &&
                        ((kick.kickLeg == Legs::left && isLeftPhase) || (kick.kickLeg == Legs::right && !isLeftPhase)) ? 1 : 0;

  if(kick.walkKickType == WalkKicks::forwardSteal)
    kick.direction = forwardStealVFeetAngle * (kick.kickLeg == Legs::left ? 1.f : -1.f) * 0.5f;

  // Get step targets
  std::vector<Pose2f> stepTargets, originalTargets;
  std::vector<Vector2f> stepSwingOffset;

  Pose2f lastExecutedStep;
  // use theOdometryData and not the updated one, because the rotation of the current frame is not added to the kickDirection yet
  getNextStepPositionsSpecialHandling(lastPhase, kick, theOdometryDataPreview, kickIndex, originalTargets, stepTargets, stepSwingOffset, lastExecutedStep);

  // Abort if needed
  Rangef maxClipX;
  Rangef maxClipY;
  Rangea maxClipRot;
  getClipRanges(maxClipX, maxClipY, maxClipRot, kick, kickIndex, kick.precision == KickPrecision::justHitTheBall);
  if(!canExecute(originalTargets, stepTargets, maxClipX, maxClipY, maxClipRot, kick.kickLeg != Legs::left, kick) || forwardStealAbortCondition(kick, stepTargets, kickIndex))
    return false;

  return true;
}

bool WalkKickEngine::findBestKickExecution(const bool canKick, const Rangea& precisionRange,
                                           const Vector2f& ballModel, WalkKickVariant& walkKickVariant,
                                           const Pose2f& lastExecutedStep, const KickPrecision precision,
                                           const bool turnKickAllowed, const bool isPreStep)
{
  if(!canKick)
    return false;

  Angle confAngle = -theKickInfo[walkKickVariant.kickType].rotationOffset;
  if(walkKickVariant.walkKickType != WalkKicks::forward || !turnKickAllowed)
  {
    walkKickVariant.kickInterpolation = 0.f;
    const DeviationValues dv = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);

    // the robot can stand further away from the ball, if the previous step was bigger.
    float xRangeMaxOffset = 0.f;
    if((walkKickVariant.walkKickType == WalkKicks::forwardLong || walkKickVariant.walkKickType == WalkKicks::forward))
    {
      Rangef maxAdditionalXRange(0.f, maxForward.max / 2.f);
      xRangeMaxOffset += maxAdditionalXRange.limit(lastExecutedStep.translation.x() + maxForwardAcceleration);
    }
    const Rangef xRange(dv.maxXDeviation.min + (isPreStep && walkKickVariant.walkKickType == WalkKicks::forwardLong ? xRangeMaxOffset / 2.f : 0.f), dv.maxXDeviation.max + xRangeMaxOffset);
    bool ballInRange = true;
    ballInRange &= dv.maxYDeviation.isInside(ballModel.y()) && xRange.isInside(ballModel.x());
    const Rangea precisionRangeWithDeviation(walkKickVariant.direction + precisionRange.min - dv.maxAngleDeviation, walkKickVariant.direction + precisionRange.max + dv.maxAngleDeviation);
    ballInRange &= precisionRangeWithDeviation.isInside(confAngle);

    if(walkKickVariant.walkKickType != WalkKicks::forward)
      clipKickDirectionWithPrecision(walkKickVariant.direction, walkKickVariant, precisionRange);

    if(walkKickVariant.walkKickType == WalkKicks::sidewardsOuter || walkKickVariant.walkKickType == WalkKicks::forwardSteal || walkKickVariant.walkKickType == WalkKicks::forwardLong)
      walkKickVariant.direction += theKickInfo[walkKickVariant.kickType].rotationOffset;

    return ballInRange;
  }
  else // for forward / turn kicks, the base relative ball pose it not one point, but a range. This can be used to find a kick angles that lets us execute the kick earlier!
  {
    const Rangea kickInterpolationAngle(walkKickVariant.direction + precisionRange.min, walkKickVariant.direction + precisionRange.max);
    const Rangef forwardRange(walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : 0_deg, walkKickVariant.kickLeg != Legs::left ? -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset : 0_deg);
    Rangef kickInterpolationRange;
    const Rangea kickAngleRange(forwardRange.limit(kickInterpolationAngle.min), forwardRange.limit(kickInterpolationAngle.max));
    kickInterpolationRange.min = Rangef::ZeroOneRange().limit(kickAngleRange.min / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset));
    kickInterpolationRange.max = Rangef::ZeroOneRange().limit(kickAngleRange.max / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset));
    walkKickVariant.kickInterpolation = kickInterpolationRange.min;
    DeviationValues dvMin = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);

    float denominator = dvMin.maxYDeviation.max - dvMin.maxYDeviation.min;
    const float ratio = Rangef::ZeroOneRange().limit((ballModel.y() - dvMin.maxYDeviation.min) / denominator); // Must be between 0 and 1
    walkKickVariant.kickInterpolation = kickInterpolationRange.min * (1.f - ratio) + kickInterpolationRange.max * ratio;
    walkKickVariant.direction = kickAngleRange.min * (1.f - ratio) + kickAngleRange.max * ratio;

    DeviationValues dvMax = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);

    dvMin.maxXDeviation.max += Rangef(0.f, maxForward.max * (1.f - kickInterpolationRange.min))
                               .limit(lastExecutedStep.translation.x() + maxForwardAcceleration);
    dvMax.maxXDeviation.max += Rangef(0.f, maxForward.max * (1.f - kickInterpolationRange.max))
                               .limit(lastExecutedStep.translation.x() + maxForwardAcceleration);

    const Rangef overallYDeviation(dvMin.maxYDeviation.min, dvMax.maxYDeviation.max);
    if(!overallYDeviation.isInside(ballModel.y()))
      return false;

    ASSERT(denominator > 0); // Must be bigger than 0
    const Rangef usedXDeviation(dvMin.maxXDeviation.min * (1.f - ratio) + dvMax.maxXDeviation.min * ratio, dvMin.maxXDeviation.max * (1.f - ratio) + dvMax.maxXDeviation.max * ratio);
    if(!usedXDeviation.isInside(ballModel.x()))
      return false;

    const Angle maxAngleDeviation = dvMin.maxAngleDeviation * (1.f - ratio) + dvMax.maxAngleDeviation * ratio;

    const Rangea precisionRangeWithDeviation(kickInterpolationAngle.min - maxAngleDeviation, kickInterpolationAngle.max + maxAngleDeviation);
    return precisionRangeWithDeviation.isInside(walkKickVariant.direction);
  }
}

bool WalkKickEngine::findBestKickExecutionDiagonal(const Rangea& precisionRange,
                                                   WalkKickVariant& walkKickVariant,
                                                   const Pose2f& lastExecutedStep, const KickPrecision precision,
                                                   const bool turnKickAllowed, const bool isPreStep)
{
  if(isPreStep)
    return false;

  walkKickVariant.diagonalKickState = WalkKickVariant::set;
  const bool isLeftPhase = walkKickVariant.kickLeg == Legs::left;
  const Pose3f& kickFootPosition = isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight;

  Pose2f scsCognition;
  const Vector2f ballPosition = getZeroBallPosition(isLeftPhase, scsCognition, walkKickVariant.ballEstimationTime);

  if((ballPosition.y() < 0.f && isLeftPhase) || (ballPosition.y() > 0.f && !isLeftPhase))
    return false;

  const Vector2f ballInOdometry = theOdometryDataPreview.inverse() * theMotionRequest.odometryData * BallPhysics::propagateBallPosition(theMotionRequest.ballEstimate.position, theMotionRequest.ballEstimate.velocity, walkKickVariant.ballEstimationTime, theBallSpecification.friction);
  const float ballInOdometryRange = (ballInOdometry - (kickFootPosition.translation.head<2>())).norm();
  const float ballInZeroRange = ballPosition.norm();
  // TODO calculate the angle based on the tip of the toe and not the origin of the foot.
  Angle kickAngleDiagonal = (ballInOdometry - kickFootPosition.translation.head<2>()).angle();

  if(walkKickVariant.walkKickType != WalkKicks::forward || !turnKickAllowed)
  {
    walkKickVariant.kickInterpolation = 0.f;
    const DeviationValues dv = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);

    // the robot can stand further away from the ball, if the previous step was bigger.
    float xRangeMaxOffset = 0.f;
    if(walkKickVariant.walkKickType == WalkKicks::forwardLong)
    {
      Rangef maxAdditionalXRange(0.f, maxForward.max / 2.f);
      xRangeMaxOffset += maxAdditionalXRange.limit(lastExecutedStep.translation.x() + maxForwardAcceleration);
    }
    const Rangef xRange(dv.maxXDeviation.min + (isPreStep && walkKickVariant.walkKickType == WalkKicks::forwardLong ? xRangeMaxOffset / 2.f : 0.f), dv.maxXDeviation.max + xRangeMaxOffset);
    bool ballInRange = xRange.min < ballInOdometryRange && // ball is far enough away from kicking foot
                       xRange.max > ballInZeroRange; // ball is probably reachable

    if(ballInRange)
    {
      const Rangea precisionRangeWithDeviation(precisionRange.min - dv.maxAngleDeviation + walkKickVariant.direction, precisionRange.max + dv.maxAngleDeviation + walkKickVariant.direction);
      if(!precisionRangeWithDeviation.isInside(kickAngleDiagonal))
      {
        const Angle missingAngle = precisionRangeWithDeviation.limit(kickAngleDiagonal) - kickAngleDiagonal;
        const Vector2f positionChange = (ballInOdometry - kickFootPosition.translation.head<2>()).rotated(missingAngle) - (ballInOdometry - kickFootPosition.translation.head<2>());
        if(std::abs(positionChange.y()) < 30.f && std::abs(positionChange.x()) < 30.f)
          kickAngleDiagonal = precisionRangeWithDeviation.limit(kickAngleDiagonal);
        else
          return false;
      }

      walkKickVariant.kickInterpolation = 0.f;
      walkKickVariant.direction = kickAngleDiagonal;
      if(walkKickVariant.walkKickType == WalkKicks::sidewardsOuter || walkKickVariant.walkKickType == WalkKicks::forwardSteal || walkKickVariant.walkKickType == WalkKicks::forwardLong)
        walkKickVariant.direction += theKickInfo[walkKickVariant.kickType].rotationOffset;
    }

    return ballInRange;
  }
  else // for forward / turn kicks, the base relative ball pose it not one point, but a range. This can be used to find a kick angles that lets us execute the kick earlier!
  {
    const Rangef forwardRange(walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : 0_deg, walkKickVariant.kickLeg != Legs::left ? -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset : 0_deg);
    Rangef kickInterpolationRange;
    const Rangea kickAngleRange(forwardRange.limit(precisionRange.min), forwardRange.limit(precisionRange.max));
    kickInterpolationRange.min = Rangef::ZeroOneRange().limit(kickAngleRange.min / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset));
    kickInterpolationRange.max = Rangef::ZeroOneRange().limit(kickAngleRange.max / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset));
    walkKickVariant.kickInterpolation = kickInterpolationRange.min;
    DeviationValues dvMin = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);
    walkKickVariant.kickInterpolation = kickInterpolationRange.max;
    DeviationValues dvMax = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);

    dvMin.maxXDeviation.max += Rangef(0.f, maxForward.max * (1.f - kickInterpolationRange.min))
                               .limit(lastExecutedStep.translation.x() + maxForwardAcceleration);
    dvMax.maxXDeviation.max += Rangef(0.f, maxForward.max * (1.f - kickInterpolationRange.max))
                               .limit(lastExecutedStep.translation.x() + maxForwardAcceleration);

    // No Y check wanted
    const Rangef usedXDeviation(std::min(dvMin.maxXDeviation.min, dvMax.maxXDeviation.min), std::max(dvMin.maxXDeviation.max, dvMax.maxXDeviation.max));
    if(usedXDeviation.min > ballInOdometryRange || // ball is far enough away from kicking foot
       usedXDeviation.max < ballInZeroRange) // ball is probably reachable
      return false;

    const Angle maxAngleDeviation = std::max(dvMin.maxAngleDeviation, dvMax.maxAngleDeviation);
    const Rangea precisionRangeWithDeviation(precisionRange.min - maxAngleDeviation + walkKickVariant.direction, precisionRange.max + maxAngleDeviation + walkKickVariant.direction);

    if(precisionRangeWithDeviation.isInside(kickAngleDiagonal))
    {
      walkKickVariant.kickInterpolation = 0.f;
      walkKickVariant.direction = kickAngleDiagonal;
      return true;
    }
    return false;
  }
}

void WalkKickEngine::getClipRanges(Rangef& maxClipX, Rangef& maxClipY, Rangea& maxClipRot,
                                   const WalkKickVariant& kick, const int kickIndex, const bool isJustHitTheBall)
{
  maxClipX = !isJustHitTheBall ? walkKicksList[kick.walkKickType].maxClipBeforeAbortX : walkKicksList[kick.walkKickType].maxClipBeforeAbortJustHitTheBallX;
  maxClipY = !isJustHitTheBall ? walkKicksList[kick.walkKickType].maxClipBeforeAbortY : walkKicksList[kick.walkKickType].maxClipBeforeAbortJustHitTheBallY;
  maxClipRot = walkKicksList[kick.walkKickType].maxClipBeforeAbortRot;
  if(kick.walkKickType == WalkKicks::forward)
  {
    const Rangef& refMaxClipX = !isJustHitTheBall ? walkKicksList[WalkKicks::turnOut].maxClipBeforeAbortX : walkKicksList[WalkKicks::turnOut].maxClipBeforeAbortJustHitTheBallX;
    const Rangef& refMaxClipY = !isJustHitTheBall ? walkKicksList[WalkKicks::turnOut].maxClipBeforeAbortY : walkKicksList[WalkKicks::turnOut].maxClipBeforeAbortJustHitTheBallY;
    const Rangea& refMaxClipRot = !isJustHitTheBall ? walkKicksList[WalkKicks::turnOut].maxClipBeforeAbortRot : walkKicksList[WalkKicks::turnOut].maxClipBeforeAbortJustHitTheBallRot;
    maxClipX.min = (1.f - kick.kickInterpolation) * maxClipX.min + kick.kickInterpolation * refMaxClipX.min;
    maxClipX.max = (1.f - kick.kickInterpolation) * maxClipX.max + kick.kickInterpolation * refMaxClipX.max;
    maxClipY.min = (1.f - kick.kickInterpolation) * maxClipY.min + kick.kickInterpolation * refMaxClipY.min;
    maxClipY.max = (1.f - kick.kickInterpolation) * maxClipY.max + kick.kickInterpolation * refMaxClipY.max;
    maxClipRot.min = (1.f - kick.kickInterpolation) * maxClipRot.min + kick.kickInterpolation * refMaxClipRot.min;
    maxClipRot.max = (1.f - kick.kickInterpolation) * maxClipRot.max + kick.kickInterpolation * refMaxClipRot.max;
  }
  else if(kick.walkKickType == WalkKicks::forwardSteal && kickIndex > 0)
  {
    maxClipX.min = -maxClipBeforeAbortForwardSteal.translation.x();
    maxClipX.max = maxClipBeforeAbortForwardSteal.translation.x();
    maxClipY.min = -maxClipBeforeAbortForwardSteal.translation.y();
    maxClipY.max = maxClipBeforeAbortForwardSteal.translation.y();
    maxClipRot.min = -maxClipBeforeAbortForwardSteal.rotation;
    maxClipRot.max = maxClipBeforeAbortForwardSteal.rotation;
  }
  if(kick.diagonalKickState == WalkKickVariant::set)
  {
    maxClipY.min *= 2.f;
    maxClipY.max *= 2.f;
  }
}

void WalkKickEngine::clipKickDirectionWithPrecision(Angle& direction, const WalkKickVariant& walkKickVariant, const Rangea& precisionRange)
{
  const Rangea precision(walkKickVariant.direction + precisionRange.min, walkKickVariant.direction + precisionRange.max);
  direction = precision.limit(-theKickInfo[walkKickVariant.kickType].rotationOffset);
}

void WalkKickEngine::clipStepTarget(Pose2f& original, Pose2f& newPose,
                                    const bool isLeftPhase, const KickKeyframePart keyframePart,
                                    const WalkKickVariant& kick, const Pose2f& lastExecutedStep)
{
  // No modification of the max speed. This should have happend in applyAllClipping. Only check if this actually happend!
  VERIFY(keyframePart.maxSideStep >= 0.f);
  if(kick.diagonalKickState != WalkKickVariant::set)
  {
    Angle currentRot = newPose.rotation;
    clipRotation(currentRot, isLeftPhase);
    ASSERT(currentRot == newPose.rotation);
  }
  ASSERT(maxForward.isInside(newPose.translation.x()));
  const Rangef maxSideStepClip((isLeftPhase ? maxSide.min : -keyframePart.maxSideStep) - 0.1f, (isLeftPhase ? keyframePart.maxSideStep : -maxSide.min) + 0.1f);
  ASSERT(maxSideStepClip.isInside(newPose.translation.y()));
  if(keyframePart.returnXToZero)
  {
    original.translation.x() = 0.f;
    newPose.translation.x() = 0.f;
  }
  if(keyframePart.returnYToZero)
  {
    original.translation.y() = 0.f;
    newPose.translation.y() = 0.f;
  }
  if(keyframePart.returnRotationToZero)
  {
    original.rotation = 0_deg;
    newPose.rotation = 0_deg;
  }
  if(keyframePart.hold)
    newPose = -lastExecutedStep;
}

void WalkKickEngine::clipRotation(Angle& rotation, const bool isLeftPhase)
{
  Rangea useTurnClip(isLeftPhase ? maxTurnLeftFoot.min : -maxTurnLeftFoot.max, isLeftPhase ? maxTurnLeftFoot.max : -maxTurnLeftFoot.min);
  useTurnClip.clamp(rotation);
}

void WalkKickEngine::getNextStepPositionsSpecialHandling(const MotionPhase& lastPhase, const WalkKickVariant& kick, const OdometryData& odometryData,
                                                         const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                                         std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep)
{
  if(kick.walkKickType == WalkKicks::forward && kick.diagonalKickState != WalkKickVariant::set)
  {
    std::vector<Pose2f> forwardStepTargets, forwardOriginalTargets, turnStepTargets, turnOriginalTargets;
    std::vector<Vector2f> forwardSwingOffset, turnSwingOffset;
    WalkKickVariant forwardKick = kick;
    WalkKickVariant turnKick = kick;
    turnKick.walkKickType = WalkKicks::Type::turnOut;

    getNextStepPositionsWithClipping(lastPhase, forwardKick, odometryData, index, forwardOriginalTargets, forwardStepTargets, forwardSwingOffset, lastExecutedStep);
    getNextStepPositionsWithClipping(lastPhase, turnKick, odometryData, index, turnOriginalTargets, turnStepTargets, turnSwingOffset, lastExecutedStep);
    for(size_t i = 0; i < forwardOriginalTargets.size(); i++)
    {
      originalTargets.emplace_back((1.f - kick.kickInterpolation) * forwardOriginalTargets[i].rotation + kick.kickInterpolation * turnOriginalTargets[i].rotation,
                                   (1.f - kick.kickInterpolation) * forwardOriginalTargets[i].translation.x() + kick.kickInterpolation * turnOriginalTargets[i].translation.x(),
                                   (1.f - kick.kickInterpolation) * forwardOriginalTargets[i].translation.y() + kick.kickInterpolation * turnOriginalTargets[i].translation.y());
      stepTargets.emplace_back((1.f - kick.kickInterpolation) * forwardStepTargets[i].rotation + kick.kickInterpolation * turnStepTargets[i].rotation,
                               (1.f - kick.kickInterpolation) * forwardStepTargets[i].translation.x() + kick.kickInterpolation * turnStepTargets[i].translation.x(),
                               (1.f - kick.kickInterpolation) * forwardStepTargets[i].translation.y() + kick.kickInterpolation * turnStepTargets[i].translation.y());
      stepSwingOffsets.emplace_back((1.f - kick.kickInterpolation) * forwardSwingOffset[i].x() + kick.kickInterpolation * turnSwingOffset[i].x(),
                                    (1.f - kick.kickInterpolation) * forwardSwingOffset[i].y() + kick.kickInterpolation * turnSwingOffset[i].y());
    }
  }
  else
    getNextStepPositionsWithClipping(lastPhase, kick, odometryData, index, originalTargets, stepTargets, stepSwingOffsets, lastExecutedStep);
}

void WalkKickEngine::applyAllClipping(const WalkKickVariant& kick, const bool isLeftPhase,
                                      const int index, std::vector<Pose2f>& stepTargets, Pose2f& lastExecutedStep)
{
  std::vector<Vector2f> translationPolygon;
  std::vector<Vector2f> backRight;
  std::vector<Vector2f> frontLeft;
  std::vector<Rangef> maxSideStepClip;

  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    backRight.emplace_back(-theWalkingEngineOutput.maxPossibleBackwardStepSize, -theWalkingEngineOutput.maxPossibleStepSize.translation.y());
    frontLeft.push_back(theWalkingEngineOutput.maxPossibleStepSize.translation);
    maxSideStepClip.emplace_back(
      isLeftPhase ? maxSide.min : std::min(-0.01f, - walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].maxSideStep),
      isLeftPhase ? std::max(0.01f, walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].maxSideStep) : -maxSide.min);
  }

  // Set min step target
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    if(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].clipMaxSpeed)
      frontLeft[i].x() = std::max(maxForwardAcceleration, std::min(frontLeft[i].x(), lastExecutedStep.translation.x() + maxForwardAcceleration));
    if(kick.precision == KickPrecision::justHitTheBall || kick.walkKickType == WalkKicks::forwardLong)
      frontLeft[i].x() = std::min(frontLeft[i].x(), forwardLongMaxStepSize);

    // clip max allowed step size
    // different than from WalkingEngine, because kicks are allowed to be different!
    frontLeft[i].x() = maxForward.limit(frontLeft[i].x());
    backRight[i].x() = maxForward.limit(backRight[i].x());
    frontLeft[i].y() = maxSideStepClip[i].limit(frontLeft[i].y());
    backRight[i].y() = maxSideStepClip[i].limit(backRight[i].y());
  }

  // TODO do not use lastExecutedStep, but the measured position of both feet.
  // This would need a convertion function from theWalkGeneratorOutput, because of the arm compensation and torso shift
  // The walkingEngine already has a convertion like this, but only internally
  auto applyHoldClip = [&](const std::size_t i)
  {
    ASSERT(i < stepTargets.size());
    if(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].holdXTranslationWhenMovingBackward)
      stepTargets[i].translation.x() = std::max(stepTargets[i].translation.x(), i == 0 ? -lastExecutedStep.translation.x() : stepTargets[i - 1].translation.x());

    if(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].holdYTranslationWhenFeetTogether)
    {
      if(isLeftPhase)
        stepTargets[i].translation.y() = std::min(stepTargets[i].translation.y(), stepTargets[stepTargets.size() - 1].translation.y());
      else
        stepTargets[i].translation.y() = std::max(stepTargets[i].translation.y(), stepTargets[stepTargets.size() - 1].translation.y());
    }
    stepTargets[i].translation.x() = maxForward.limit(stepTargets[i].translation.x());
    stepTargets[i].translation.y() = maxSideStepClip[i].limit(stepTargets[i].translation.y());
  };

  for(std::size_t i = 0; i < stepTargets.size(); i++)
  {
    theWalkGenerator.generateTranslationPolygon(isLeftPhase, 0, Pose2f(1.f, 1.f, 1.f), translationPolygon, backRight[i], frontLeft[i], true, true);

    // Make sure the point (0,0) is always part of the polygon
    ASSERT(Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), Vector2f(0.f, 0.f)));

    if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), stepTargets[i].translation))
    {
      Vector2f p1(0.f, 0.f);
      VERIFY((Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f),
                                                              stepTargets[i].translation.normalized(0.1f)), p1)));
      stepTargets[i].translation = p1;
    }
    clipRotation(stepTargets[i].rotation, isLeftPhase);
  }
  // Apply special clipping afterwards. Those clipping can not be applied with the translation polygon
  // Also it needs to be after all other clipping as the last step keyframe is used here
  for(std::size_t i = 0; i < stepTargets.size(); i++)
    applyHoldClip(i);
}

void WalkKickEngine::getNextStepPositionsWithClipping(const MotionPhase& lastPhase, const WalkKickVariant& kick, const OdometryData& odometryData,
                                                      const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                                      std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep)
{
  // Get step targets
  const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());

  if(kick.diagonalKickState == WalkKickVariant::set)
  {
    Pose2f leftPose;
    Pose2f rightPose;
    std::tie(leftPose, rightPose, lastExecutedStep) = theWalkGenerator.getStartOffsetOfNextWalkPhase(lastPhase);
    Pose2f scsCognition;
    const Vector2f ballPosition = getZeroBallPosition(isLeftPhase, scsCognition, kick.ballEstimationTime);
    getNextStepPositionsDiagonal(lastPhase, originalTargets, stepTargets, stepSwingOffsets, kick, index, ballPosition, lastExecutedStep.rotation);

    applyAllClipping(kick, isLeftPhase, index, stepTargets, lastExecutedStep);
    std::vector<Pose2f> placeHolder = stepTargets; // Do not override the original step targets again
    for(size_t i = 0; i < stepTargets.size(); i++)
    {
      clipStepTarget(placeHolder[i], stepTargets[i], isLeftPhase,
                     walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i], kick, lastExecutedStep);
    }
    return;
  }

  // Get ball position
  Pose2f scsCognition;
  const Vector2f ballPosition = getZeroBallPosition(isLeftPhase, scsCognition, kick.ballEstimationTime);

  // Get complete odometry for kick direction
  Pose2f leftPose;
  Pose2f rightPose;
  std::tie(leftPose, rightPose, lastExecutedStep) = theWalkGenerator.getStartOffsetOfNextWalkPhase(lastPhase);
  Angle convertedOdometryRotation = scsCognition.rotation - (theOdometryDataPreview.rotation - odometryData.rotation);
  getNextStepPositions(lastPhase, originalTargets, stepTargets, stepSwingOffsets, kick, index,
                       ballPosition, convertedOdometryRotation, isLeftPhase ? leftPose : rightPose);

  applyAllClipping(kick, isLeftPhase, index, stepTargets, lastExecutedStep);
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    clipStepTarget(originalTargets[i], stepTargets[i], isLeftPhase,
                   walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i], kick, lastExecutedStep);
  }
}

void WalkKickEngine::getNextStepPositions(const MotionPhase& lastPhase, std::vector<Pose2f>& original, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                                          const WalkKickVariant& kick, const int index, const Vector2f& ballPosition,
                                          const Angle convertedOdometryRotation, const Pose2f& lastExecutedSupportStep)
{
  if(index >= static_cast<int>(walkKicksList[kick.walkKickType].kickKeyFrame.size()))
    return;
  const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());
  const bool isPreStep = (kick.kickLeg == Legs::left) != isLeftPhase;
  const float sign = ((isLeftPhase && !isPreStep) || (!isLeftPhase && isPreStep)) ? 1.f : -1.f;

  // Calculate each target
  std::vector<Pose2f> stepTargets; // Step targets for walk phase
  std::vector<Vector2f> stepTargetsWithoutOffset; // Step targets for walk phase

  Vector2f minStepOffset(0.f, 0.f);
  bool skipRecalculation = false;
  for(int i = 0; i < static_cast<int>(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes.size()); i++)
  {
    // Get position relative to the ball
    Vector2f stepTargetPositionRelativToBall = getPowerScaledRPB(kick.walkKickType, index, i, kick.power) + minStepOffset;
    stepTargetPositionRelativToBall.y() *= sign;

    Vector2f stepTargetPositionRelativToBallWithOffset = stepTargetPositionRelativToBall - getPowerScaledOSF(kick.walkKickType, index, i, kick.power);
    Angle waitRotationPosition = walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].directionRatio * (kick.direction + convertedOdometryRotation);

    clipRotation(waitRotationPosition, isLeftPhase); // Use the rotation, that will be used at the end after the clipping. Otherwise the target position can be off

    stepTargetPositionRelativToBall.rotate(waitRotationPosition);
    stepTargetPositionRelativToBallWithOffset.rotate(waitRotationPosition);
    const Pose2f target = Pose2f(waitRotationPosition, ballPosition + stepTargetPositionRelativToBall); // But save the original planned rotation, so the check if the step is possible still checks for the rotation
    const Pose2f targetWithOffset = Pose2f(waitRotationPosition, ballPosition + stepTargetPositionRelativToBallWithOffset);

    bool recalculate = false;

    auto shiftMinStepX = [&](const float xTarget)
    {
      minStepOffset.x() = xTarget - targetWithOffset.translation.x();
      minStepOffset.y() = minStepOffset.x() * std::tan(waitRotationPosition);
      recalculate = true;
    };

    if(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].relativeBackwardStep &&
       lastExecutedSupportStep.translation.x() + walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].relativeBackwardStep.value()
       <= targetWithOffset.translation.x() + 0.1f)
      shiftMinStepX(lastExecutedSupportStep.translation.x() + walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].relativeBackwardStep.value());

    if(targetWithOffset.translation.x() + 0.1f < walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.x())
      shiftMinStepX(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.x());

    if((kick.kickLeg == Legs::left && isLeftPhase && targetWithOffset.translation.y() + 0.1f < walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()) ||
       (kick.kickLeg != Legs::left && isLeftPhase && targetWithOffset.translation.y() + 0.1f < -walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()) ||
       (kick.kickLeg == Legs::left && !isLeftPhase && targetWithOffset.translation.y() - 0.1f > walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()) ||
       (kick.kickLeg != Legs::left && !isLeftPhase && targetWithOffset.translation.y() - 0.1f > -walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()))
    {
      minStepOffset.y() = (kick.kickLeg != Legs::left ? -1.f : 1.f) * walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y() - targetWithOffset.translation.y();
      minStepOffset.x() = std::tan(waitRotationPosition) / minStepOffset.y();
      recalculate = true;
    }

    if(recalculate && !skipRecalculation)
    {
      original.push_back(targetWithOffset);
      minStepOffset.rotate(-waitRotationPosition);
      i--;
      skipRecalculation = true;
      continue;
    }
    if(!skipRecalculation)
      original.push_back(targetWithOffset);

    skipRecalculation = false;
    minStepOffset = Vector2f(0.f, 0.f);

    // Get step target
    stepTargets.push_back(targetWithOffset);
    stepTargetsWithoutOffset.push_back(target.translation - targetWithOffset.translation);
  }

  poses = stepTargets;
  stepSwingOffsets = stepTargetsWithoutOffset;
  ASSERT(poses.size() == original.size());
}

void WalkKickEngine::getNextStepPositionsDiagonal(const MotionPhase& lastPhase, std::vector<Pose2f>& original, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                                                  const WalkKickVariant& kick, const int index, const Vector2f& ballPosition,
                                                  const Angle lastStepRotation)
{
  if(index >= static_cast<int>(walkKicksList[kick.walkKickType].kickKeyFrame.size()))
    return;
  const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());
  const float sign = isLeftPhase ? 1.f : -1.f; // diagonal kick has no pre step

  // Calculate each target
  std::vector<Pose2f> stepTargets; // Step targets for walk phase
  std::vector<Vector2f> stepTargetsWithoutOffset; // Step targets for walk phase

  bool skipRecalculation = false;
  Vector2f minStepOffset(0.f, 0.f);
  for(int i = 0; i < static_cast<int>(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes.size()); i++)
  {
    // Get position relative to the ball
    Vector2f stepTargetPositionRelativToBall = getPowerScaledRPB(kick.walkKickType, index, i, kick.power) + minStepOffset;
    stepTargetPositionRelativToBall.y() *= sign;

    Vector2f stepTargetPositionRelativToBallWithOffset = stepTargetPositionRelativToBall - getPowerScaledOSF(kick.walkKickType, index, i, kick.power);
    Angle waitRotationPosition = walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].directionRatio * kick.direction;

    clipRotation(waitRotationPosition, isLeftPhase); // Use the rotation, that will be used at the end after the clipping. Otherwise the target position can be off

    stepTargetPositionRelativToBall.rotate(waitRotationPosition);
    stepTargetPositionRelativToBallWithOffset.rotate(waitRotationPosition);
    const Pose2f target = Pose2f(lastStepRotation, ballPosition + stepTargetPositionRelativToBall); // But save the original planned rotation, so the check if the step is possible still checks for the rotation
    const Pose2f targetWithOffset = Pose2f(lastStepRotation, ballPosition + stepTargetPositionRelativToBallWithOffset);

    bool recalculate = false;
    if(targetWithOffset.translation.x() + 0.1f < walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.x())
    {
      minStepOffset.x() = walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.x() - targetWithOffset.translation.x();
      minStepOffset.y() = minStepOffset.x() * std::tan(waitRotationPosition);

      recalculate = true;
    }

    if((kick.kickLeg == Legs::left && isLeftPhase && targetWithOffset.translation.y() + 0.1f < walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()) ||
       (kick.kickLeg != Legs::left && isLeftPhase && targetWithOffset.translation.y() + 0.1f < -walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()) ||
       (kick.kickLeg == Legs::left && !isLeftPhase && targetWithOffset.translation.y() - 0.1f > walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()) ||
       (kick.kickLeg != Legs::left && !isLeftPhase && targetWithOffset.translation.y() - 0.1f > -walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y()))
    {
      minStepOffset.y() = (kick.kickLeg != Legs::left ? -1.f : 1.f) * walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y() - targetWithOffset.translation.y();
      minStepOffset.x() = std::tan(waitRotationPosition) / minStepOffset.y();
      recalculate = true;
    }

    if(recalculate && !skipRecalculation)
    {
      original.push_back(targetWithOffset);
      minStepOffset.rotate(-waitRotationPosition);
      i--;
      skipRecalculation = true;
      continue;
    }
    else if(!skipRecalculation)
      original.push_back(targetWithOffset);

    skipRecalculation = false;
    minStepOffset = Vector2f(0.f, 0.f);

    // Get step target
    stepTargets.push_back(targetWithOffset);
    stepTargetsWithoutOffset.push_back(target.translation - targetWithOffset.translation);
  }

  poses = stepTargets;
  stepSwingOffsets = stepTargetsWithoutOffset;
  ASSERT(poses.size() == original.size());
}

bool WalkKickEngine::canExecute(const std::vector<Pose2f>& originalPoses, const std::vector<Pose2f>& clippedPoses, const Rangef& maxClipBeforeAbortX,
                                const Rangef& maxClipBeforeAbortY, const Rangea& maxClipBeforeAbortRot, const bool mirror, const WalkKickVariant& kick)
{
  ASSERT(originalPoses.size() == clippedPoses.size());
  bool reachable = true;
  Rangef useMaxClipBeforeAbortY = maxClipBeforeAbortY;
  if(mirror)
  {
    useMaxClipBeforeAbortY.min = -maxClipBeforeAbortY.max;
    useMaxClipBeforeAbortY.max = -maxClipBeforeAbortY.min;
  }

  if(kick.walkKickType == WalkKicks::forward && kick.shiftTurnKickPose)
  {
    float kickPoseShiftY = 0.f;
    if(mirror)
    {
      float ballOffsetY = (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[KickInfo::walkTurnRightFootToLeft].ballOffset.y();
      float ballOffsetYShifed = (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[KickInfo::walkTurnRightFootToLeftShifted].ballOffset.y();
      kickPoseShiftY = ballOffsetY - ballOffsetYShifed;
    }
    else
    {
      float ballOffsetY = (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[KickInfo::walkTurnLeftFootToRight].ballOffset.y();
      float ballOffsetYShifed = (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[KickInfo::walkTurnLeftFootToRightShifted].ballOffset.y();
      kickPoseShiftY = ballOffsetYShifed - ballOffsetY;
    }

    useMaxClipBeforeAbortY.min += std::min(kickPoseShiftY, 0.f);
    useMaxClipBeforeAbortY.max += std::max(kickPoseShiftY, 0.f);
  }

  for(size_t i = 0; i < originalPoses.size(); i++)
  {
    Pose2f diffStepTarget(0_deg, 0.f, 0.f);
    diffStepTarget.rotation = originalPoses[i].rotation - clippedPoses[i].rotation;
    diffStepTarget.translation = originalPoses[i].translation - clippedPoses[i].translation;
    if(!maxClipBeforeAbortX.contains(diffStepTarget.translation.x()) || !useMaxClipBeforeAbortY.contains(diffStepTarget.translation.y()) || !maxClipBeforeAbortRot.contains(diffStepTarget.rotation))
      reachable = false;
  }
  return reachable;
}

bool WalkKickEngine::forwardStealAbortCondition(const WalkKickVariant& kick, const std::vector<Pose2f>& steps, const int index)
{
  if(kick.walkKickType == WalkKicks::forwardSteal && theFrameInfo.getTimeSince(theMotionRequest.ballTimeWhenLastSeen) < timeSinceBallLastSeenThreshold && steps.size() >= 2)
  {
    float yDiff = 0.f;
    if(index == 0)
    {
      const Vector2f hipOffset(0.f, kick.kickLeg == Legs::left ? theRobotDimensions.yHipOffset : -theRobotDimensions.yHipOffset);
      const Vector2f forwardAndSide = (steps[1].translation + hipOffset).rotated(-steps[1].rotation * 0.5f) - 2.f * hipOffset + hipOffset.rotated(steps[1].rotation * 0.5f);
      const float sign = kick.kickLeg == Legs::left ? 1.f : -1.f;
      yDiff = ((kick.kickLeg != Legs::left ? theRobotModel.soleRight.translation.y() : theRobotModel.soleLeft.translation.y()) - (forwardAndSide.y() / 2.f + theRobotDimensions.yHipOffset * sign)) * sign;
    }
    // Get ball position
    const Pose3f supportInTorso3D = theTorsoMatrix * (kick.kickLeg != Legs::left ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f scsCognition = supportInTorso.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
    const Vector2f ballPosition = scsCognition * BallPhysics::propagateBallPosition(theMotionRequest.ballEstimate.position, theMotionRequest.ballEstimate.velocity, kick.ballEstimationTime, theBallSpecification.friction);
    if((kick.kickLeg == Legs::right && ballPosition.y() < 0.f)
       || (kick.kickLeg == Legs::left && ballPosition.y() > 0.f)
       || ballPosition.x() > forwardStealMinXAbortThreshold
       || yDiff > forwardStealMaxPreviousYTranslation)
      return true;
  }
  return false;
}

std::unique_ptr<MotionPhase> WalkKickEngine::createPhaseWithNextPhase(const MotionPhase& lastPhase)
{
  // No InWalkKicks after standing
  if(lastPhase.type != MotionPhase::walk)
    return std::unique_ptr<MotionPhase>();

  // InWalkKick is over
  if(kickIndex >= static_cast<int>(walkKicksList[currentKick.walkKickType].kickKeyFrame.size()))
    return std::unique_ptr<MotionPhase>();

  // There was a false support switch in the current InWalkKick. Abort it.
  if(theWalkGenerator.wasLastPhaseInWalkKick(lastPhase) && theWalkGenerator.wasLastPhaseLeftPhase(lastPhase) == theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f()))
  {
    ANNOTATION("WalkKickEngine", "Wrong support foot switch");
    return std::unique_ptr<MotionPhase>();
  }

  if(forwardStealAbortCondition(currentKick, std::vector<Pose2f>(), kickIndex))
    return std::unique_ptr<MotionPhase>();

  // Ball was not seen for a too long time
  if(currentKick.walkKickType == WalkKicks::forwardSteal && theFrameInfo.getTimeSince(theMotionRequest.ballTimeWhenLastSeen) > timeSinceBallLastSeenThreshold && kickIndex >= 4)
    return std::unique_ptr<MotionPhase>();

  if(currentKick.walkKickType == WalkKicks::forwardSteal && kickIndex > 0)
    currentKick.direction *= -1.f;

  std::vector<Pose2f> originalTargets, stepTargets;
  std::vector<Vector2f> stepSwingOffset;
  Pose2f lastExecutedStep;
  getNextStepPositionsSpecialHandling(lastPhase, currentKick, odometryAtStart, kickIndex, originalTargets, stepTargets, stepSwingOffset, lastExecutedStep);

  // Abort if needed
  Rangef maxClipX;
  Rangef maxClipY;
  Rangea maxClipRot;
  getClipRanges(maxClipX, maxClipY, maxClipRot, currentKick, kickIndex, currentKick.precision == KickPrecision::justHitTheBall);

  if(!canExecute(originalTargets, stepTargets, maxClipX, maxClipY, maxClipRot, currentKick.kickLeg != Legs::left, currentKick))
  {
    if(kickIndex > 0 && !theWalkGenerator.wasLastPhaseInWalkKick(lastPhase) && (currentKick.walkKickType == WalkKicks::forward || currentKick.walkKickType == WalkKicks::forwardLong))
      OUTPUT_ERROR("WalkKickEngine: kick was aborted that should not have been aborted!");
    return std::unique_ptr<MotionPhase>();
  }

  // Calc ratios for the step targets
  std::vector<float> stepRatio;
  std::vector<float> timings;
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    const float timeFactor = stepDuration / 1000.f;
    // Calculate ratio based on how much the feet will move
    const Pose2f& from = i == 0 ? -lastExecutedStep : stepTargets[i - 1];
    const Pose2f& to = stepTargets[i];
    const Pose2f dif = to - from;
    const std::vector<float> timingsWaitPosition = { std::abs(dif.rotation) / (90_deg),
                                                     std::abs(dif.translation.x()) / (maxSpeed.translation.x() * timeFactor),
                                                     std::abs(dif.translation.y()) / (maxSpeed.translation.y() * timeFactor * 2.f)
                                                   };
    const float time = *std::max_element(std::begin(timingsWaitPosition), std::end(timingsWaitPosition));
    // forward / turn interpolation
    if(currentKick.walkKickType == WalkKicks::forward)
    {
      float forwardKickTimeWeighted = (walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio ? walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio : time) * (1.f - currentKick.kickInterpolation);
      float turnKickTimeWeighted = (walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio ? walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio : time) * currentKick.kickInterpolation;
      timings.emplace_back(std::max(0.0001f, forwardKickTimeWeighted + turnKickTimeWeighted));
    }
    // default case, use default value
    else if(walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio)
      timings.emplace_back(walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio);
    else
      timings.emplace_back(time > 0 ? time : 0.0001f);
  }

  // Set ratios for all step targets
  float sum = 0.f;
  for(const auto& n : timings)
    sum += n;
  for(size_t i = 0; i < timings.size(); i++)
    stepRatio.emplace_back(timings[i] / sum);

  // Set parameters for the step targets
  const Legs::Leg swingLeg = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f()) ? Legs::left : Legs::right;
  std::vector<WalkKickStep::StepKeyframe> stepKeyframes;
  for(int i = 0; i < static_cast<int>(stepTargets.size()); i++)
  {
    WalkKickStep::StepKeyframe keyframe;
    keyframe.stepTarget = stepTargets[i];
    keyframe.stepTargetSwing = stepTargets[i];
    keyframe.stepTargetSwing.translation += stepSwingOffset[i];
    keyframe.stepRatio = stepRatio[i];
    if(currentKick.walkKickType == WalkKicks::forward)
    {
      const float useSpeedUpSwingFactorForward = (1.f - currentKick.power) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionScaling.min +
                                                 currentKick.power * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionScaling.max;
      const float useSpeedUpSwingFactorTurnOut = (1.f - currentKick.power) * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionScaling.min +
                                                 currentKick.power * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionScaling.max;
      keyframe.speedUpSwing = useSpeedUpSwingFactorTurnOut * currentKick.kickInterpolation +
                              useSpeedUpSwingFactorForward * (1.f - currentKick.kickInterpolation);
      keyframe.holdXSupportTarget = currentKick.kickInterpolation < 0.5f ?
                                    walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].holdXSupportTarget :
                                    walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].holdXSupportTarget;
      keyframe.holdXSwingTarget = currentKick.kickInterpolation < 0.5f ?
                                  walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].holdXSwingTarget :
                                  walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].holdXSwingTarget;
    }
    else
    {
      keyframe.speedUpSwing = (1.f - currentKick.power) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionScaling.min +
                              currentKick.power * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionScaling.max;
      keyframe.holdXSupportTarget = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].holdXSupportTarget;
      keyframe.holdXSwingTarget = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].holdXSwingTarget;
    }

    // Set the interpolation types
    // Prestep -> smooth interpolation
    // Kickstep -> kick with linear interpolation the ball
    if(currentKick.kickLeg == swingLeg)
    {
      if(stepTargets.size() == 1 || i > 0)
        keyframe.interpolationType = WalkKickStep::InterpolationType::linear;
      else if(i == 0)
        keyframe.interpolationType = WalkKickStep::InterpolationType::cosinusZeroToMax;
    }
    else
    {
      if(stepTargets.size() == 1)
        keyframe.interpolationType = WalkKickStep::InterpolationType::normal;
      else
      {
        if(i == 0)
          keyframe.interpolationType = WalkKickStep::InterpolationType::cosinusZeroToMax;
        else if(i < static_cast<int>(stepTargets.size()) - 1)
          keyframe.interpolationType = WalkKickStep::InterpolationType::linear;
        else
          keyframe.interpolationType = WalkKickStep::InterpolationType::sinusMaxToZero;
      }
    }

    stepKeyframes.emplace_back(keyframe);
  }
  WalkKickStep kickStep;
  kickStep.keyframe = stepKeyframes;
  kickStep.increaseSwingHeightFactor = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].increaseSwingHeightFactor;
  WalkKicks::Type jointOffsetsKick = currentKick.walkKickType;
  if(currentKick.walkKickType == WalkKicks::forward && currentKick.kickInterpolation > 0.5f)
    jointOffsetsKick = WalkKicks::turnOut;
  std::vector<WalkKickStep::LongKickParams> jointOffsets = walkKicksList[jointOffsetsKick].kickKeyFrame[kickIndex].jointOffsets;

  // For the forwardLong, we want to compensate the ankle pitch of the kick foot, to prevent a ball lob which would result in a large kick direction deviation
  if(currentKick.walkKickType == WalkKicks::forwardLong && kickIndex == 1)
  {
    ASSERT(stepTargets.size() > 0);
    const float firstStepTargetTranslationX = stepTargets[0].translation.x();
    const float factor = Rangef::ZeroOneRange().limit((firstStepTargetTranslationX - forwardLongFirstStepAnkleCompensationRange.min) / (forwardLongFirstStepAnkleCompensationRange.max - forwardLongFirstStepAnkleCompensationRange.min));
    jointOffsets.emplace_back();
    jointOffsets.back().joint = Joints::lAnklePitch; // Joint is mirrored in the WalkingEngine
    jointOffsets.back().offset = factor * forwardLongFirstStepAnkleCompensationAngle; // offset value
    jointOffsets.back().minRatio = 0.f; // immediately apply offset
    jointOffsets.back().middleRatio = 0.5f; // before ball is touched
    jointOffsets.back().maxRatio = 0.75f; // offset should be zero before support foot switch
    jointOffsets.back().minimumRatio = 0.f;
    jointOffsets.back().shiftByKeyFrame = -1; // no shift
  }
  kickStep.longKickParams = jointOffsets;

  kickStep.overrideOldSwingFoot = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].overrideOldSwingFoot;
  kickStep.overrideOldSupportFoot = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].overrideOldSupportFoot;
  if(currentKick.walkKickType == WalkKicks::forward)
  {
    kickStep.numOfBalanceSteps = static_cast<int>(std::round(static_cast<float>(walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].numOfBalanceSteps * (1.f - currentKick.kickInterpolation)) +
                                                             static_cast<float>(walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].numOfBalanceSteps * currentKick.kickInterpolation)));;

    kickStep.reduceSwingFootHeight = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].reduceSwingFootHeight * (1.f - currentKick.kickInterpolation) +
                                     walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].reduceSwingFootHeight * currentKick.kickInterpolation;
  }
  else
  {
    kickStep.numOfBalanceSteps = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].numOfBalanceSteps;
    kickStep.reduceSwingFootHeight = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].reduceSwingFootHeight;
  }
  for(WalkKickStep::LongKickParams& offsets : kickStep.longKickParams)
  {
    if(offsets.shiftByKeyFrame >= 0)
    {
      float shift = 0.f;
      for(size_t i = 0; i < kickStep.keyframe.size() && i <= static_cast<size_t>(offsets.shiftByKeyFrame); i++)
        shift += kickStep.keyframe[offsets.shiftByKeyFrame].stepRatio;
      if(shift > offsets.minimumRatio)
      {
        float ratioShift = shift - offsets.minRatio;
        offsets.maxRatio += ratioShift;
        offsets.middleRatio += ratioShift;
        offsets.minRatio += ratioShift;
      }
    }
  }
  kickStep.currentKick = currentKick.walkKickType;
  kickStep.useSlowSupportFootHeightAfterKickInterpolation = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useSlowSupportFootHeightAfterKickInterpolation;
  if(currentKick.walkKickType == WalkKicks::forward)
    kickStep.useSlowSwingFootHeightInterpolation = (1.f - currentKick.kickInterpolation) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useSlowSwingFootHeightInterpolation +
                                                   currentKick.kickInterpolation * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].useSlowSwingFootHeightInterpolation;
  else
    kickStep.useSlowSwingFootHeightInterpolation = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useSlowSwingFootHeightInterpolation;
  kickStep.useLastKeyframeForSupportFootXTranslation = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useLastKeyframeForSupportFootXTranslation;

  // create WalkPhase
  kickIndex++;
  auto phase = theWalkGenerator.createPhaseWithNextPhase(kickStep,
                                                         lastPhase, std::bind(&WalkKickEngine::createPhaseWithNextPhase, this, std::placeholders::_1), currentKick.delayParams.kickIndex == kickIndex - 1 ? currentKick.delayParams.delay : 0.f);

  phase->kickType = lastPhase.kickType;
  return phase;
}

DeviationValues WalkKickEngine::getForwardTurnKickDeviation(const WalkKickVariant& kick, const Pose2f& lastStep,
                                                            const KickPrecision precision, const bool isPreStep)
{
  // if the robot moved backward last step, we currently do not want to scale.
  // TODO test if it even matters
  Pose2f filteredLastStep = lastStep;
  filteredLastStep.translation.x() = std::max(0.f, filteredLastStep.translation.x());
  const float hipPoseOffset = kick.kickLeg == Legs::left ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset;
  DeviationValues dv;

  if(kick.walkKickType == WalkKicks::forward)
  {
    // Get the other kick
    const WalkKicks::Type otherWalkKick = WalkKicks::Type::turnOut;

    KickInfo::KickType otherKick;

    if(kick.shiftTurnKickPose)
      otherKick = kick.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRightShifted : KickInfo::walkTurnRightFootToLeftShifted;
    else
      otherKick = kick.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;

    // first get the interpolated deviations
    dv.maxAngleDeviation = (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].maxAngleDeviation + kick.kickInterpolation * walkKicksList[otherWalkKick].maxAngleDeviation;
    dv.maxXDeviation = Rangef((1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].maxXDeviation.min + kick.kickInterpolation * walkKicksList[otherWalkKick].maxXDeviation.min,
                              (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].maxXDeviation.max + kick.kickInterpolation * walkKicksList[otherWalkKick].maxXDeviation.max);
    dv.maxYDeviation = Rangef((1.f - kick.kickInterpolation) * (walkKicksList[kick.walkKickType].maxYDeviation.min + walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.min * filteredLastStep.translation.x()) + kick.kickInterpolation * (walkKicksList[otherWalkKick].maxYDeviation.min + walkKicksList[otherWalkKick].sideDeviationScalingBasedOnLastStep.min * filteredLastStep.translation.x()),
                              (1.f - kick.kickInterpolation) * (walkKicksList[kick.walkKickType].maxYDeviation.max + walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.max * filteredLastStep.translation.x()) + kick.kickInterpolation * (walkKicksList[otherWalkKick].maxYDeviation.max + walkKicksList[otherWalkKick].sideDeviationScalingBasedOnLastStep.max * filteredLastStep.translation.x()));

    // isPreStep is used here, because otherwise the pre step will result in a strange huge walk step, which will not kick the ball like expected
    if(precision == KickPrecision::notPrecise || (isPreStep && precision == KickPrecision::justHitTheBall))
    {
      dv.maxXDeviation.min += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalXDeviation.min + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalXDeviation.min;
      dv.maxXDeviation.max += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalXDeviation.max + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalXDeviation.max;
      dv.maxYDeviation.min += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalYDeviation.min + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalYDeviation.min;
      dv.maxYDeviation.max += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalYDeviation.max + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalYDeviation.max;
    }
    else if(precision == KickPrecision::justHitTheBall)
    {
      dv.maxXDeviation.min += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalXDeviationJustHitTheBall.min + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalXDeviationJustHitTheBall.min;
      dv.maxXDeviation.max += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalXDeviationJustHitTheBall.max + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalXDeviationJustHitTheBall.max;
      dv.maxYDeviation.min += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalYDeviationJustHitTheBall.min + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalYDeviationJustHitTheBall.min;
      dv.maxYDeviation.max += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalYDeviationJustHitTheBall.max + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalYDeviationJustHitTheBall.max;
    }

    // mirror if kickleg is not the left one
    if(kick.kickLeg != Legs::left)
      dv.maxYDeviation = Rangef(-dv.maxYDeviation.max, -dv.maxYDeviation.min);

    // add ball offset
    dv.maxXDeviation.min += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.x() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.x();
    dv.maxXDeviation.max += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.x() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.x();

    if(kick.shiftTurnKickPose)
    {
      const auto normalOtherKick = kick.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;
      const float otherY = (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.y();
      const float normalOtherY = (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[normalOtherKick].ballOffset.y();
      dv.maxYDeviation.min += std::min(otherY, normalOtherY);
      dv.maxYDeviation.max += std::max(otherY, normalOtherY);
    }
    else
    {
      dv.maxYDeviation.min += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.y();
      dv.maxYDeviation.max += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.y();
    }

    // kick pose is based on the robot pose, not the pose of the ball relative to the kick foot
    dv.maxYDeviation.min += hipPoseOffset;
    dv.maxYDeviation.max += hipPoseOffset;

    return dv;
  }

  // first get the deviations
  dv.maxAngleDeviation = walkKicksList[kick.walkKickType].maxAngleDeviation;
  dv.maxXDeviation = walkKicksList[kick.walkKickType].maxXDeviation;
  dv.maxYDeviation = walkKicksList[kick.walkKickType].maxYDeviation;
  dv.maxYDeviation.min += walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.min * filteredLastStep.translation.x();
  dv.maxYDeviation.max += walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.max * filteredLastStep.translation.x();

  // isPreStep is used here, because otherwise the pre step will result in a strange huge walk step, which will not kick the ball like expected
  if(precision == KickPrecision::notPrecise || (isPreStep && precision == KickPrecision::justHitTheBall))
  {
    dv.maxXDeviation.min += walkKicksList[kick.walkKickType].additionalXDeviation.min;
    dv.maxXDeviation.max += walkKicksList[kick.walkKickType].additionalXDeviation.max;
    dv.maxYDeviation.min += walkKicksList[kick.walkKickType].additionalYDeviation.min;
    dv.maxYDeviation.max += walkKicksList[kick.walkKickType].additionalYDeviation.max;
  }
  else if(precision == KickPrecision::justHitTheBall)
  {
    dv.maxXDeviation.min += walkKicksList[kick.walkKickType].additionalXDeviationJustHitTheBall.min;
    dv.maxXDeviation.max += walkKicksList[kick.walkKickType].additionalXDeviationJustHitTheBall.max;
    dv.maxYDeviation.min += walkKicksList[kick.walkKickType].additionalYDeviationJustHitTheBall.min;
    dv.maxYDeviation.max += walkKicksList[kick.walkKickType].additionalYDeviationJustHitTheBall.max;
  }

  // mirror if kickleg is not the left one
  if(kick.kickLeg != Legs::left)
    dv.maxYDeviation = Rangef(-dv.maxYDeviation.max, -dv.maxYDeviation.min);

  // add ball offset
  if(kick.walkKickType == WalkKicks::forwardSteal)
  {
    dv.maxXDeviation.min -= forwardStealWaitingKickPose.x();
    dv.maxXDeviation.max -= forwardStealWaitingKickPose.x();
  }
  else
  {
    dv.maxXDeviation.min -= theKickInfo[kick.kickType].ballOffset.x();
    dv.maxXDeviation.max -= theKickInfo[kick.kickType].ballOffset.x();
  }
  dv.maxYDeviation.min -= theKickInfo[kick.kickType].ballOffset.y();
  dv.maxYDeviation.max -= theKickInfo[kick.kickType].ballOffset.y();

  // kick pose is based on the robot pose, not the pose of the ball relative to the kick foot
  dv.maxYDeviation.min += hipPoseOffset;
  dv.maxYDeviation.max += hipPoseOffset;

  return dv;
}

Vector2f WalkKickEngine::getPowerScaledRPB(const WalkKicks::Type kickType, const int index, const int keyframe, const float power)
{
  return walkKicksList[kickType].kickKeyFrame[index].keyframes[keyframe].relativePositionToBallMax * power + (1.f - power) * walkKicksList[kickType].kickKeyFrame[index].keyframes[keyframe].relativePositionToBallMin;
}

Vector2f WalkKickEngine::getPowerScaledOSF(const WalkKicks::Type kickType, const int index, const int keyframe, const float power)
{
  return walkKicksList[kickType].kickKeyFrame[index].keyframes[keyframe].offsetSwingFootMax * power + (1.f - power) * walkKicksList[kickType].kickKeyFrame[index].keyframes[keyframe].offsetSwingFootMin;
}

Vector2f WalkKickEngine::getZeroBallPosition(const bool isLeftPhase, Pose2f& scsCognition, const float ballTime)
{
  const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
  const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
  const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
  scsCognition = hipOffset * hipOffset * supportInTorso.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
  return scsCognition * BallPhysics::propagateBallPosition(theMotionRequest.ballEstimate.position, theMotionRequest.ballEstimate.velocity, ballTime, theBallSpecification.friction);
}

Pose2f WalkKickEngine::getVShapeWalkStep(const bool isLeftPhase, const Angle direction)
{
  Pose2f scsCognition;
  Vector2f ball = getZeroBallPosition(isLeftPhase, scsCognition, 0.f);
  Vector2f offset = forwardStealWaitingKickPose;
  if(!isLeftPhase)
    offset.y() *= -1.f;

  const Angle poseRot = (isLeftPhase ? 1.f : -1.f) * forwardStealVFeetAngle / 2.f + scsCognition.rotation + direction;
  offset.rotate(-direction);
  Pose2f step(poseRot, ball + offset);
  if(isLeftPhase == (step.translation.y() < 0.f))
    step.translation.y() = 0.f;
  return step;
}

void WalkKickEngine::draw(const Pose2f& pose)
{
  if(!enableDrawings)
    return;

  // Stuff below is to debug all calculated later
  const Pose3f supportInTorso3D = theTorsoMatrix;
  const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
  const Pose2f scsCognition = supportInTorso.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
  const Vector2f ballPosition = scsCognition * theMotionRequest.ballEstimate.position;

  // Calculate where the support foot will be placed
  const bool isRightSupport = theFootSupport.support < 0.f;
  Pose3f nextSupport = Pose3f((isRightSupport ? theRobotModel.soleRight : theRobotModel.soleLeft).translation).rotateZ((isRightSupport ? theRobotModel.soleRight : theRobotModel.soleLeft).rotation.getZAngle());
  nextSupport.translate(0.f, isRightSupport ? theRobotDimensions.yHipOffset : -theRobotDimensions.yHipOffset, 0.f);
  nextSupport.translate(Vector3f(pose.translation.x(), pose.translation.y(), 0.f));
  nextSupport.rotateZ(pose.rotation);
  nextSupport.translate(0.f, isRightSupport ? theRobotDimensions.yHipOffset : -theRobotDimensions.yHipOffset, 0.f);

  Vector3f sole = theRobotModel.soleLeft.translation;
  POINT3D("module:WalkKickEngine:nextSupport", ballPosition.x(), ballPosition.y(), -theTorsoMatrix.translation.z(), 10, ColorRGBA::blue);
  const Vector3f upperLeft = (nextSupport * Vector3f(100.f, isRightSupport ? -38.f : -50.f, 0.f));
  const Vector3f upperRight = (nextSupport * Vector3f(100.f, isRightSupport ? 50.f : 38.f, 0.f));
  const Vector3f lowerLeft = (nextSupport * Vector3f(-50.f, isRightSupport ? -38.f : -50.f, 0.f));
  const Vector3f lowerRight = (nextSupport * Vector3f(-50.f, isRightSupport ? 50.f : 38.f, 0.f));
  const float height = -theTorsoMatrix.translation.z();
  LINE3D("module:WalkKickEngine:nextSupport", upperLeft.x(), upperLeft.y(), height, upperRight.x(), upperRight.y(), height, 4, ColorRGBA::red);
  LINE3D("module:WalkKickEngine:nextSupport", upperLeft.x(), upperLeft.y(), height, lowerLeft.x(), lowerLeft.y(), height, 4, ColorRGBA::red);
  LINE3D("module:WalkKickEngine:nextSupport", lowerRight.x(), lowerRight.y(), height, upperRight.x(), upperRight.y(), height, 4, ColorRGBA::red);
  LINE3D("module:WalkKickEngine:nextSupport", lowerRight.x(), lowerRight.y(), height, lowerLeft.x(), lowerLeft.y(), height, 4, ColorRGBA::red);
}
