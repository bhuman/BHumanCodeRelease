/**
 * @file WalkKickEngine.cpp
 *
 * This file implements a module that provides a walk kick generator.
 *
 * @author Philip Reichenberg
 */

#include "WalkKickEngine.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Streams/TypeRegistry.h"
#include <regex>
#include <cmath>

MAKE_MODULE(WalkKickEngine, motionControl);

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
  ASSERT(walkKicksList[WalkKicks::forwardSteal].kickKeyFrame.size() == 6);
}

void WalkKickEngine::update(WalkKickGenerator& walkKickGenerator)
{
  updatedOdometryData = theOdometryData;
  updatedOdometryData.rotation = Rotation::Euler::getZAngle(theInertialData.orientation3D);

  walkKickGenerator.drawStep = [this](const Pose2f& step)
  {
    draw(step);
  };

  walkKickGenerator.getVShapeWalkStep = [this](const bool isLeftPhase, const Angle direction)
  {
    return getVShapeWalkStep(isLeftPhase, direction);
  };

  walkKickGenerator.canStart = [this](const WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const Rangea& precisionRange,
                                      const bool alignPrecisely, const bool preStepAllowed, const bool turnKickAllowed, const float kickPoseShiftY)
  {
    // No InWalkKicks after standing
    if(lastPhase.type != MotionPhase::walk)
      return false;

    // After an InWalkKick at least one normal walk phase must follow
    if(theWalkGenerator.wasLastPhaseInWalkKick(lastPhase))
      return false;

    // Get last step size
    Pose2f leftPose(0_deg, 0.f, 0.f);
    Pose2f rightPose(0_deg, 0.f, 0.f);
    Pose2f lastExecutedStep(0_deg, 0.f, 0.f);
    std::tie(leftPose, rightPose, lastExecutedStep) = theWalkGenerator.getStartOffsetOfNextWalkPhase(lastPhase);
    const Pose2f lastStepChange = theWalkGenerator.getLastStepChange(lastPhase);
    bool criticalLastStepYTranslation = std::abs(lastStepChange.translation.y()) > restrictedYTranslationOfPreStep.y();
    if(walkKickVariant.walkKickType == WalkKicks::forward)
    {
      const Rangef forwardRange(walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : 0_deg, walkKickVariant.kickLeg != Legs::left ? -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset : 0_deg);
      const Angle confAngle = forwardRange.limit(walkKickVariant.direction);
      const float interpolation = Rangef::ZeroOneRange().limit(confAngle / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset) / 0.5f);
      criticalLastStepYTranslation |= std::abs(lastStepChange.translation.x()) > turnKickPreviousMaxXStepSize * (1.f - interpolation) + restrictedYTranslationOfPreStep.x() * interpolation;
    }
    else if(walkKickVariant.walkKickType == WalkKicks::turnOut)
      criticalLastStepYTranslation |= std::abs(lastStepChange.translation.x()) > restrictedYTranslationOfPreStep.x();
    else
      criticalLastStepYTranslation |= std::abs(lastStepChange.translation.x()) > restrictedYTranslationOfPreStep.x();

    if(alignPrecisely && (walkKickVariant.walkKickType == WalkKicks::forward || walkKickVariant.walkKickType == WalkKicks::turnOut)) // force the robot to stop before the kick
      criticalLastStepYTranslation |= std::abs(lastStepChange.translation.x()) > forwardTurnPreciseMaxStepSize;

    // Is left the swing foot?
    bool isLeftPhase = theWalkGenerator.isNextLeftPhase(walkKickVariant.kickLeg == Legs::left, lastPhase);

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
    const Pose2f scsCognition = supportInTorso.inverse() * updatedOdometryData.inverse() * theMotionRequest.odometryData;
    const Vector2f ballModel = scsCognition * theMotionRequest.ballEstimate.position;

    // Can the kick be executed based on the swing foot and the kick type?
    const bool preStepIsNext = walkKickVariant.kickLeg != (isLeftPhase ? Legs::left : Legs::right);
    bool canKick = preStepIsNext && (preStepAllowed || walkKickVariant.walkKickType == WalkKicks::forwardSteal);
    if(!preStepIsNext && ((walkKickVariant.walkKickType == WalkKicks::forward && std::abs(walkKickVariant.direction) < forwardPreStepSkipMaxKickAngle) || walkKickVariant.walkKickType == WalkKicks::forwardLong))
      canKick = true;
    if(walkKickVariant.walkKickType == WalkKicks::sidewardsOuter)
      canKick = !preStepIsNext && lastStepChange.translation.x() < sidewardsOuterForwardRestriction;

    // was last step size too big?
    // this check is needed to ensure that the ball is hit
    if(criticalLastStepYTranslation)
      for(WalkKicks::Type kickType : walkKicksWithRestrictedStart)
        if(kickType == walkKickVariant.walkKickType)
          canKick = false;

    // Can the kick be executed based on the ball position relative to the kick foot?
    Angle confAngle = -theKickInfo[walkKickVariant.kickType].rotationOffset;
    WalkKickVariant kick = walkKickVariant;
    if(walkKickVariant.walkKickType == WalkKicks::forward && turnKickAllowed)
    {
      const Rangef forwardRange(walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : 0_deg, walkKickVariant.kickLeg != Legs::left ? -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset : 0_deg);
      confAngle = forwardRange.limit(walkKickVariant.direction);
      kick.kickInterpolation = Rangef::ZeroOneRange().limit(confAngle / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset));
    }

    const DeviationValues dv = getForwardTurnKickDeviation(kick, lastExecutedStep, alignPrecisely, kickPoseShiftY);

    float xRangeMaxOffset = 0.f;
    // the turn kick / the interpolation version between the forward and the turn kick might need this offset too.
    // TODO eval if the robot can execute the kick without falling, with the offsets
    if((walkKickVariant.walkKickType == WalkKicks::forwardLong || walkKickVariant.walkKickType == WalkKicks::forward))
    {
      Rangef maxAdditionalXRange(0.f, maxForward.max * (1.f - kick.kickInterpolation)); // for turn kicks, we do not want to start them far away
      // the robot can stand further away from the ball, if the previous step was bigger.
      xRangeMaxOffset += maxAdditionalXRange.limit(lastExecutedStep.translation.x() + maxForwardAcceleration);
    }

    const Vector2f xRange(dv.maxXDeviation.min, dv.maxXDeviation.max + xRangeMaxOffset);
    bool ballInRange = true;
    ballInRange &= ballModel.y() > dv.maxYDeviation.min && ballModel.y() < dv.maxYDeviation.max && ballModel.x() > xRange.x() && ballModel.x() < xRange.y();
    const Rangea precisionRangeWithDeviation(precisionRange.min - dv.maxAngleDeviation, precisionRange.max + dv.maxAngleDeviation);
    ballInRange &= precisionRangeWithDeviation.isInside(confAngle - walkKickVariant.direction);
    canKick &= ballInRange;
    if(canKick)
    {
      // Can the resulting step target be executed?
      const int kickIndex = (kick.walkKickType == WalkKicks::forward || kick.walkKickType == WalkKicks::forwardLong) &&
                            ((kick.kickLeg == Legs::left && isLeftPhase) || (kick.kickLeg == Legs::right && !isLeftPhase)) ? 1 : 0;

      // Get step targets
      std::vector<Pose2f> stepTargets, originalTargets;
      std::vector<Vector2f> stepSwingOffset;

      if(kick.walkKickType != WalkKicks::forward)
        clipKickDirectionWithPrecision(kick.direction, walkKickVariant, precisionRange);

      if(kick.walkKickType == WalkKicks::sidewardsOuter || kick.walkKickType == WalkKicks::forwardSteal || kick.walkKickType == WalkKicks::forwardLong)
        kick.direction += theKickInfo[kick.kickType].rotationOffset;

      Pose2f lastExecutedStep;
      // use theOdometryData and not the updated one, because the rotation of the current frame is not added to the kickDirection yet
      getNextStepPositionsSpecialHandling(lastPhase, kick, theOdometryData, kickIndex, originalTargets, stepTargets, stepSwingOffset, lastExecutedStep);

      // for the forwardLong, we want to check where the ball will lie after the prestep.
      if((kick.walkKickType == WalkKicks::forwardLong) &&
         std::abs(walkKickVariant.direction) < forwardPreStepSkipMaxKickAngle && (kick.kickLeg == Legs::left) != isLeftPhase)
      {
        const Vector2f xRange(dv.maxXDeviation.min + stepTargets[stepTargets.size() - 1].translation.x(), dv.maxXDeviation.max + stepTargets[stepTargets.size() - 1].translation.x());
        if(!(ballModel.x() > xRange.x() && ballModel.x() < xRange.y()))
          return false;
      }

      // Abort if needed
      Rangef maxClipX = walkKicksList[kick.walkKickType].maxClipBeforAbortX;
      Rangef maxClipY = walkKicksList[kick.walkKickType].maxClipBeforAbortY;
      Rangea maxClipRot = walkKicksList[kick.walkKickType].maxClipBeforAbortRot;
      if(kick.walkKickType == WalkKicks::forward)
      {
        const Rangef& refMaxClipX = walkKicksList[WalkKicks::turnOut].maxClipBeforAbortX;
        const Rangef& refMaxClipY = walkKicksList[WalkKicks::turnOut].maxClipBeforAbortY;
        const Rangea& refMaxClipRot = walkKicksList[WalkKicks::turnOut].maxClipBeforAbortRot;
        maxClipX.min = (1.f - kick.kickInterpolation) * maxClipX.min + kick.kickInterpolation * refMaxClipX.min;
        maxClipX.max = (1.f - kick.kickInterpolation) * maxClipX.max + kick.kickInterpolation * refMaxClipX.max;
        maxClipY.min = (1.f - kick.kickInterpolation) * maxClipY.min + kick.kickInterpolation * refMaxClipY.min;
        maxClipY.max = (1.f - kick.kickInterpolation) * maxClipY.max + kick.kickInterpolation * refMaxClipY.max;
        maxClipRot.min = (1.f - kick.kickInterpolation) * maxClipRot.min + kick.kickInterpolation * refMaxClipRot.min;
        maxClipRot.max = (1.f - kick.kickInterpolation) * maxClipRot.max + kick.kickInterpolation * refMaxClipRot.max;
      }
      if(!canExecute(originalTargets, stepTargets, maxClipX, maxClipY, maxClipRot, kickPoseShiftY, kick.kickLeg != Legs::left) || forwardStealAbortCondition(kick))
        return false;
    }
    if(walkKickVariant.walkKickType == WalkKicks::forwardSteal)
      return canKick && std::abs((theTorsoMatrix * theRobotModel.soleLeft).rotation.getZAngle() - (theTorsoMatrix * theRobotModel.soleRight).rotation.getZAngle()) > forwardStealMinVAngleThreshold;
    return canKick;
  };

  walkKickGenerator.createPhase = [this](const WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const Rangea& precisionRange, const bool playSound, const float kickPoseShiftY)
  {
    currentKickPoseShiftY = kickPoseShiftY;
    odometryAtStart = theOdometryData; // use theOdometryData and not the updated one, because the rotation of the current frame is not added on the kick direction yet
    currentKick = walkKickVariant;

    if(currentKick.walkKickType == WalkKicks::forward)
    {
      const Rangef forwardRange(walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : 0_deg, walkKickVariant.kickLeg != Legs::left ? -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset : 0_deg);
      const Angle confAngle = forwardRange.limit(walkKickVariant.direction);
      currentKick.kickInterpolation = Rangef::ZeroOneRange().limit(confAngle / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset));
    }
    else
      clipKickDirectionWithPrecision(currentKick.direction, walkKickVariant, precisionRange);

    if(currentKick.walkKickType == WalkKicks::sidewardsOuter || currentKick.walkKickType == WalkKicks::forwardSteal || currentKick.walkKickType == WalkKicks::forwardLong)   // Hack for the side kick
      currentKick.direction += theKickInfo[walkKickVariant.kickType].rotationOffset;
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false, lastPhase);

    // Special handling for the forward and forwardLong kicks
    kickIndex = ((currentKick.walkKickType == WalkKicks::forward && std::abs(currentKick.direction) < forwardPreStepSkipMaxKickAngle) || currentKick.walkKickType == WalkKicks::forwardLong) &&
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

void WalkKickEngine::clipKickDirectionWithPrecision(Angle& direction, const WalkKickVariant& walkKickVariant, const Rangea& precisionRange)
{
  const Rangea precision(walkKickVariant.direction + precisionRange.min, walkKickVariant.direction + precisionRange.max);
  direction = precision.limit(-theKickInfo[walkKickVariant.kickType].rotationOffset);
}

void WalkKickEngine::clipStepTarget(Pose2f& original, Pose2f& newPose, const Pose2f& lastStep, const bool isLeftPhase, const KickKeyframePart keyframePart, const WalkKicks::Type walkKickType)
{
  newPose = original;
  Rangef useSideTurn(isLeftPhase ? maxSide.min : -maxSide.max, isLeftPhase ? maxSide.max : -maxSide.min);
  VERIFY(keyframePart.maxSideStep >= 0.f);
  Rangef maxSideStepClip(isLeftPhase ? maxSide.min : -keyframePart.maxSideStep, isLeftPhase ? keyframePart.maxSideStep : -maxSide.min);
  clipRotation(newPose.rotation, isLeftPhase);
  maxForward.clamp(newPose.translation.x());
  useSideTurn.clamp(newPose.translation.y());
  maxSideStepClip.clamp(newPose.translation.y());
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
  if(keyframePart.clipMaxSpeed)
  {
    if(newPose.translation.x() > lastStep.translation.x() + maxForwardAcceleration)
      newPose.translation.x() = lastStep.translation.x() + maxForwardAcceleration;
  }
  if(walkKickType == WalkKicks::forwardLong)
    newPose.translation.x() = std::min(newPose.translation.x(), forwardLongMaxStepSize);
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
  if(kick.walkKickType == WalkKicks::forward)
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
                                    (1.f - currentKick.kickInterpolation) * forwardSwingOffset[i].y() + kick.kickInterpolation * turnSwingOffset[i].y());
    }
  }
  else
    getNextStepPositionsWithClipping(lastPhase, kick, odometryData, index, originalTargets, stepTargets, stepSwingOffsets, lastExecutedStep);
}

void WalkKickEngine::applyAllClipping(const WalkKickVariant& kick, const bool isLeftPhase,
                                      const int index, std::vector<Pose2f>& stepTargets, Pose2f& lastExecutedStep)
{
  // Set min step target
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    stepTargets[i].translation.x() = std::max(stepTargets[i].translation.x(), walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.x());
    if(kick.kickLeg == Legs::left && isLeftPhase)
      stepTargets[i].translation.y() = std::max(stepTargets[i].translation.y(), walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y());
    else if(kick.kickLeg == Legs::left && !isLeftPhase)
      stepTargets[i].translation.y() = std::min(stepTargets[i].translation.y(), walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y());
    else if(kick.kickLeg != Legs::left && isLeftPhase)
      stepTargets[i].translation.y() = std::max(stepTargets[i].translation.y(), -walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y());
    else if(kick.kickLeg != Legs::left && !isLeftPhase)
      stepTargets[i].translation.y() = std::min(stepTargets[i].translation.y(), -walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].minStepTarget.y());
  }

  // TODO do not use lastExecutedStep, but the measured position of both feet.
  // This would need a convertion function from theWalkGeneratorOutput, because of the arm compensation and torso shift
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    if(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].holdXTranslationWhenMovingBackward)
    {
      if(i == 0 && stepTargets[i].translation.x() < -lastExecutedStep.translation.x())
        stepTargets[i].translation.x() = -lastExecutedStep.translation.x();
      else if(i > 0 && stepTargets[i].translation.x() < stepTargets[i - 1].translation.x())
        stepTargets[i].translation.x() = stepTargets[i - 1].translation.x();
    }
    if(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].holdYTranslationWhenFeetTogether)
    {
      if(i == 0 && std::abs(stepTargets[i].translation.y()) < std::abs(lastExecutedStep.translation.y()))
        stepTargets[i].translation.y() = -lastExecutedStep.translation.y();
      else if(i > 0 && std::abs(stepTargets[i].translation.y()) < std::abs(stepTargets[i - 1].translation.y()))
        stepTargets[i].translation.y() = stepTargets[i - 1].translation.y();
    }
    if(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].hold)
      stepTargets[i] = -lastExecutedStep;
  }
}

void WalkKickEngine::getNextStepPositionsWithClipping(const MotionPhase& lastPhase, const WalkKickVariant& kick, const OdometryData& odometryData,
                                                      const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                                      std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep)
{
  // Get step targets
  const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false, lastPhase);

  // Get ball position
  Pose2f scsCognition;
  const Vector2f ballPosition = getZeroBallPosition(isLeftPhase, scsCognition);

  // Get complete odometry for kick direction
  Angle convertedOdometryRotation = scsCognition.rotation - (updatedOdometryData.rotation - odometryData.rotation);
  std::vector<Pose2f> clipOffsets;
  getNextStepPositions(lastPhase, stepTargets, stepSwingOffsets, kick, index, ballPosition, clipOffsets, convertedOdometryRotation);
  Pose2f leftPose;
  Pose2f rightPose;
  std::tie(leftPose, rightPose, lastExecutedStep) = theWalkGenerator.getStartOffsetOfNextWalkPhase(lastPhase);
  std::vector<Pose2f> beforAllClipping = stepTargets;
  originalTargets = stepTargets;
  applyAllClipping(kick, isLeftPhase, index, stepTargets, lastExecutedStep);
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    clipStepTarget(originalTargets[i], stepTargets[i], lastExecutedStep, isLeftPhase, walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i], kick.walkKickType);
    if(kick.walkKickType == WalkKicks::forwardLong && i == 0 && index == 1)
    {
      stepTargets[i].translation.y() = (stepTargets[i].translation.y() - (-lastExecutedStep.translation.y())) / 2.f + (-lastExecutedStep.translation.y());
      originalTargets[i].translation.y() = stepTargets[i].translation.y();
    }
    Pose2f clip = Pose2f(beforAllClipping[i].rotation - stepTargets[i].rotation, stepTargets[i].translation - ballPosition);
    clip.translation = clip.translation.rotate(-beforAllClipping[i].rotation);
    clip.translation += getPowerScaledOSF(kick.walkKickType, index, static_cast<int>(i), kick.power);
    clipOffsets.push_back(clip);
  }

  getNextStepPositions(lastPhase, stepTargets, stepSwingOffsets, kick, index, ballPosition, clipOffsets, convertedOdometryRotation);
  applyAllClipping(kick, isLeftPhase, index, stepTargets, lastExecutedStep);
  std::vector<Pose2f> placeHolder = stepTargets; // Do not override the original step targets again
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    clipStepTarget(placeHolder[i], stepTargets[i], lastExecutedStep, isLeftPhase, walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i], kick.walkKickType);
  }
}

void WalkKickEngine::getNextStepPositions(const MotionPhase& lastPhase, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                                          const WalkKickVariant& kick, const int index, const Vector2f ballPosition,
                                          std::vector<Pose2f> clipOffsets, const Angle convertedOdometryRotation)
{
  if(index >= static_cast<int>(walkKicksList[kick.walkKickType].kickKeyFrame.size()))
    return;
  const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false, lastPhase);
  const bool isPreStep = (kick.kickLeg == Legs::left) != isLeftPhase;
  const float sign = ((isLeftPhase && !isPreStep) || (!isLeftPhase && isPreStep)) ? 1.f : -1.f;

  // Calculate each target
  std::vector<Pose2f> stepTargets; // Step targets for walk phase
  std::vector<Vector2f> stepTargetsWithoutOffset; // Step targets for walk phase

  for(int i = 0; i < static_cast<int>(walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes.size()); i++)
  {
    // Get position relative to the ball
    Vector2f stepTargetPositionRelativToBall = getPowerScaledRPB(kick.walkKickType, index, i, kick.power);
    stepTargetPositionRelativToBall.y() *= sign;
    if(clipOffsets.size() > 0)
    {
      if(kick.walkKickType == WalkKicks::sidewardsOuter)
        stepTargetPositionRelativToBall.y() = clipOffsets[i].translation.y();
      else
        stepTargetPositionRelativToBall.x() = clipOffsets[i].translation.x();
      if(kick.walkKickType == WalkKicks::forwardLong && index == 1)
        stepTargetPositionRelativToBall.y() = clipOffsets[i].translation.y();
    }
    Vector2f stepTargetPositionRelativToBallWithOffset = stepTargetPositionRelativToBall - getPowerScaledOSF(kick.walkKickType, index, i, kick.power);

    Angle waitRotationPosition = walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].directionRatio * (kick.direction + convertedOdometryRotation);

    if(clipOffsets.size() > 0)
      waitRotationPosition -= clipOffsets[i].rotation;

    clipRotation(waitRotationPosition, isLeftPhase); // Use the rotation, that will be used at the end after the clipping. Otherwise the target position can be off

    stepTargetPositionRelativToBall.rotate(waitRotationPosition);
    stepTargetPositionRelativToBallWithOffset.rotate(waitRotationPosition);
    Pose2f target = Pose2f(waitRotationPosition, ballPosition + stepTargetPositionRelativToBall); // But save the original planned rotation, so the check if the step is possible still checks for the rotation
    Pose2f targetWithOffset = Pose2f(waitRotationPosition, ballPosition + stepTargetPositionRelativToBallWithOffset);

    // Get step target
    stepTargets.emplace_back(targetWithOffset);
    stepTargetsWithoutOffset.emplace_back(target.translation - targetWithOffset.translation);
  }

  poses = stepTargets;
  stepSwingOffsets = stepTargetsWithoutOffset;
}

bool WalkKickEngine::canExecute(const std::vector<Pose2f>& originalPoses, const std::vector<Pose2f>& clippedPoses, const Rangef& maxClipBeforAbortX,
                                const Rangef& maxClipBeforAbortY, const Rangea& maxClipBeforAbortRot, const float kickPoseShiftY, const bool mirror)
{
  bool reachable = true;
  Rangef useMaxClipBeforAbortY = maxClipBeforAbortY;
  if(mirror)
  {
    useMaxClipBeforAbortY.min = -maxClipBeforAbortY.max;
    useMaxClipBeforAbortY.max = -maxClipBeforAbortY.min;
  }
  useMaxClipBeforAbortY.min += std::min(-kickPoseShiftY, 0.f);
  useMaxClipBeforAbortY.max += std::max(-kickPoseShiftY, 0.f);
  for(size_t i = 0; i < originalPoses.size(); i++)
  {
    Pose2f diffStepTarget(0_deg, 0.f, 0.f);
    diffStepTarget.rotation = originalPoses[i].rotation - clippedPoses[i].rotation;
    diffStepTarget.translation = originalPoses[i].translation - clippedPoses[i].translation;
    if(!maxClipBeforAbortX.contains(diffStepTarget.translation.x()) || !useMaxClipBeforAbortY.contains(diffStepTarget.translation.y()) || !maxClipBeforAbortRot.contains(diffStepTarget.rotation))
      reachable = false;
  }
  return reachable;
}

bool WalkKickEngine::forwardStealAbortCondition(const WalkKickVariant& kick)
{
  if(kick.walkKickType == WalkKicks::forwardSteal && theFrameInfo.getTimeSince(theMotionRequest.ballTimeWhenLastSeen) < timeSinceBallLastSeenThreshold)
  {
    // Get ball position
    const Pose3f supportInTorso3D = theTorsoMatrix * (kick.kickLeg != Legs::left ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f scsCognition = supportInTorso.inverse() * updatedOdometryData.inverse() * theMotionRequest.odometryData;
    const Vector2f ballPosition = scsCognition * theMotionRequest.ballEstimate.position;
    if((kick.kickLeg == Legs::right && ballPosition.y() < 0.f)
       || (kick.kickLeg == Legs::left && ballPosition.y() > 0.f)
       || ballPosition.x() > forwardStealMinXAbortThreshold)
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
  if(theWalkGenerator.wasLastPhaseInWalkKick(lastPhase) && theWalkGenerator.wasLastPhaseLeftPhase(lastPhase) == theWalkGenerator.isNextLeftPhase(false, lastPhase))
  {
    ANNOTATION("WalkKickEngine", "Wrong support foot switch");
    return std::unique_ptr<MotionPhase>();
  }

  if(forwardStealAbortCondition(currentKick))
    return std::unique_ptr<MotionPhase>();

  // Ball was not seen for a too long time
  if(currentKick.walkKickType == WalkKicks::forwardSteal && theFrameInfo.getTimeSince(theMotionRequest.ballTimeWhenLastSeen) > timeSinceBallLastSeenThreshold && kickIndex >= 4)
    return std::unique_ptr<MotionPhase>();

  std::vector<Pose2f> originalTargets, stepTargets;
  std::vector<Vector2f> stepSwingOffset;
  Pose2f lastExecutedStep;
  getNextStepPositionsSpecialHandling(lastPhase, currentKick, odometryAtStart, kickIndex, originalTargets, stepTargets, stepSwingOffset, lastExecutedStep);

  // Abort if needed
  Rangef maxClipX = walkKicksList[currentKick.walkKickType].maxClipBeforAbortX;
  Rangef maxClipY = walkKicksList[currentKick.walkKickType].maxClipBeforAbortY;
  Rangea maxClipRot = walkKicksList[currentKick.walkKickType].maxClipBeforAbortRot;
  if(currentKick.walkKickType == WalkKicks::forward)
  {
    const Rangef& refMaxClipX = walkKicksList[WalkKicks::turnOut].maxClipBeforAbortX;
    const Rangef& refMaxClipY = walkKicksList[WalkKicks::turnOut].maxClipBeforAbortY;
    const Rangea& refMaxClipRot = walkKicksList[WalkKicks::turnOut].maxClipBeforAbortRot;
    maxClipX.min = (1.f - currentKick.kickInterpolation) * maxClipX.min + currentKick.kickInterpolation * refMaxClipX.min;
    maxClipX.max = (1.f - currentKick.kickInterpolation) * maxClipX.max + currentKick.kickInterpolation * refMaxClipX.max;
    maxClipY.min = (1.f - currentKick.kickInterpolation) * maxClipY.min + currentKick.kickInterpolation * refMaxClipY.min;
    maxClipY.max = (1.f - currentKick.kickInterpolation) * maxClipY.max + currentKick.kickInterpolation * refMaxClipY.max;
    maxClipRot.min = (1.f - currentKick.kickInterpolation) * maxClipRot.min + currentKick.kickInterpolation * refMaxClipRot.min;
    maxClipRot.max = (1.f - currentKick.kickInterpolation) * maxClipRot.max + currentKick.kickInterpolation * refMaxClipRot.max;
  }
  else if(currentKick.walkKickType == WalkKicks::forwardSteal && kickIndex > 0)
  {
    maxClipX.min = -maxClipBeforAbortForwardSteal.translation.x();
    maxClipX.max = maxClipBeforAbortForwardSteal.translation.x();
    maxClipY.min = -maxClipBeforAbortForwardSteal.translation.y();
    maxClipY.max = maxClipBeforAbortForwardSteal.translation.y();
    maxClipRot.min = -maxClipBeforAbortForwardSteal.rotation;
    maxClipRot.max = maxClipBeforAbortForwardSteal.rotation;
  }
  if(!canExecute(originalTargets, stepTargets, maxClipX, maxClipY, maxClipRot, currentKickPoseShiftY, currentKick.kickLeg != Legs::left))
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
    // Calculate ratio based on how much the feet will move
    const Pose2f& from = i == 0 ? -lastExecutedStep : stepTargets[i - 1];
    const Pose2f& to = stepTargets[i];
    const Pose2f dif = to - from;
    const std::vector<float> timingsWaitPosition = { std::abs(dif.rotation) / maxSpeed.rotation,
                                                     std::abs(dif.translation.x()) / maxSpeed.translation.x(),
                                                     std::abs(dif.translation.y()) / maxSpeed.translation.y()
                                                   };
    const float time = *std::max_element(std::begin(timingsWaitPosition), std::end(timingsWaitPosition));
    // forward / turn interpolation
    if(currentKick.walkKickType == WalkKicks::forward)
    {
      float timeFactor = 1000.f / stepDuration;
      float forwardKickTimeWeighted = (walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio ? walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio* timeFactor : time) * (1.f - currentKick.kickInterpolation);
      float turnKickTimeWeighted = (walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio ? walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio* timeFactor : time) * currentKick.kickInterpolation;
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
    stepKeyframes.emplace_back(keyframe);
  }
  WalkKickStep kickStep;
  kickStep.keyframe = stepKeyframes;
  kickStep.increaseSwingHeightFactor = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].increaseSwingHeightFactor;
  std::vector<WalkKickStep::LongKickParams> jointOffsets = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].jointOffsets;

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
  kickStep.useSlowFootHeightInterpolation = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useSlowFootHeightInterpolation;

  // create WalkPhase
  kickIndex++;
  auto phase = theWalkGenerator.createPhaseWithNextPhase(kickStep,
                                                         lastPhase, std::bind(&WalkKickEngine::createPhaseWithNextPhase, this, std::placeholders::_1));

  phase->kickType = lastPhase.kickType;
  return phase;
}

DeviationValues WalkKickEngine::getForwardTurnKickDeviation(const WalkKickVariant& kick, const Pose2f& lastStep, const bool alignPrecisely, const float kickPoseShiftY)
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
    const KickInfo::KickType otherKick = kick.kickLeg == Legs::left ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;

    // first get the interpolated deviations
    dv.maxAngleDeviation = (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].maxAngleDeviation + kick.kickInterpolation * walkKicksList[otherWalkKick].maxAngleDeviation;
    dv.maxXDeviation = Rangef((1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].maxXDeviation.min + kick.kickInterpolation * walkKicksList[otherWalkKick].maxXDeviation.min,
                              (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].maxXDeviation.max + kick.kickInterpolation * walkKicksList[otherWalkKick].maxXDeviation.max);
    dv.maxYDeviation = Rangef((1.f - kick.kickInterpolation) * (walkKicksList[kick.walkKickType].maxYDeviation.min + walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.min * filteredLastStep.translation.x()) + kick.kickInterpolation * (walkKicksList[otherWalkKick].maxYDeviation.min + walkKicksList[otherWalkKick].sideDeviationScalingBasedOnLastStep.min * filteredLastStep.translation.x()),
                              (1.f - kick.kickInterpolation) * (walkKicksList[kick.walkKickType].maxYDeviation.max + walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.max * filteredLastStep.translation.x()) + kick.kickInterpolation * (walkKicksList[otherWalkKick].maxYDeviation.max + walkKicksList[otherWalkKick].sideDeviationScalingBasedOnLastStep.max * filteredLastStep.translation.x()));

    if(!alignPrecisely)
    {
      dv.maxXDeviation.min += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalXDeviation.min + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalXDeviation.min;
      dv.maxXDeviation.max += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalXDeviation.max + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalXDeviation.max;
      dv.maxYDeviation.min += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalYDeviation.min + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalYDeviation.min;
      dv.maxYDeviation.max += (1.f - kick.kickInterpolation) * walkKicksList[kick.walkKickType].additionalYDeviation.max + kick.kickInterpolation * walkKicksList[otherWalkKick].additionalYDeviation.max;
    }

    // mirror if kickleg is not the left one
    if(kick.kickLeg != Legs::left)
      dv.maxYDeviation = Rangef(-dv.maxYDeviation.max, -dv.maxYDeviation.min);

    // add ball offset
    dv.maxXDeviation.min += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.x() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.x();
    dv.maxXDeviation.max += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.x() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.x();
    dv.maxYDeviation.min += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.y();
    dv.maxYDeviation.max += (1.f - kick.kickInterpolation) * -theKickInfo[kick.kickType].ballOffset.y() + kick.kickInterpolation * -theKickInfo[otherKick].ballOffset.y();

    // kick pose is based on the robot pose, not the pose of the ball relative to the kick foot
    dv.maxYDeviation.min += hipPoseOffset;
    dv.maxYDeviation.max += hipPoseOffset;

    dv.maxYDeviation.min -= kickPoseShiftY;
    dv.maxYDeviation.max -= kickPoseShiftY;

    return dv;
  }

  // first get the deviations
  dv.maxAngleDeviation = walkKicksList[kick.walkKickType].maxAngleDeviation;
  dv.maxXDeviation = walkKicksList[kick.walkKickType].maxXDeviation;
  dv.maxYDeviation = walkKicksList[kick.walkKickType].maxYDeviation;
  dv.maxYDeviation.min += walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.min * filteredLastStep.translation.x();
  dv.maxYDeviation.max += walkKicksList[kick.walkKickType].sideDeviationScalingBasedOnLastStep.max * filteredLastStep.translation.x();

  if(!alignPrecisely)
  {
    dv.maxXDeviation.min += walkKicksList[kick.walkKickType].additionalXDeviation.min;
    dv.maxXDeviation.max += walkKicksList[kick.walkKickType].additionalXDeviation.max;
    dv.maxYDeviation.min += walkKicksList[kick.walkKickType].additionalYDeviation.min;
    dv.maxYDeviation.max += walkKicksList[kick.walkKickType].additionalYDeviation.max;
  }

  // mirror if kickleg is not the left one
  if(kick.kickLeg != Legs::left)
    dv.maxYDeviation = Rangef(-dv.maxYDeviation.max, -dv.maxYDeviation.min);

  // add ball offset
  if(kick.walkKickType == WalkKicks::forwardSteal)
  {
    dv.maxXDeviation.min -= forwardStealKickPose.x();
    dv.maxXDeviation.max -= forwardStealKickPose.x();
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

  dv.maxYDeviation.min -= kickPoseShiftY;
  dv.maxYDeviation.max -= kickPoseShiftY;

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

Vector2f WalkKickEngine::getZeroBallPosition(const bool isLeftPhase, Pose2f& scsCognition)
{
  const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
  const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
  const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
  scsCognition = hipOffset * hipOffset * supportInTorso.inverse() * updatedOdometryData.inverse() * theMotionRequest.odometryData;
  return scsCognition * theMotionRequest.ballEstimate.position;
}

Pose2f WalkKickEngine::getVShapeWalkStep(const bool isLeftPhase, const Angle direction)
{
  Pose2f scsCognition;
  const Vector2f ball = getZeroBallPosition(isLeftPhase, scsCognition);
  Vector2f offset;
  if(std::abs(scsCognition.rotation) < forwardStealVFeetAngle * forwardStealVFeetAngleFactor)
    offset = forwardStealWaitingKickPose;
  else
    offset = forwardStealKickPose;
  if(!isLeftPhase)
    offset.y() *= -1.f;
  offset.rotate(direction + (isLeftPhase ? forwardStealVFeetAngle : -forwardStealVFeetAngle) + scsCognition.rotation);
  Pose2f step(direction + (isLeftPhase ? forwardStealVFeetAngle : -forwardStealVFeetAngle) + scsCognition.rotation, ball + offset);
  step.translation.x() = std::max(0.f, std::min(walkToBallForForwardStealMaxXTranslation, step.translation.x()));
  return step;
}

void WalkKickEngine::draw(const Pose2f& pose)
{
  if(!enableDrawings)
    return;

  // Stuff below is to debug all calculated later
  const Pose3f supportInTorso3D = theTorsoMatrix;
  const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
  const Pose2f scsCognition = supportInTorso.inverse() * updatedOdometryData.inverse() * theMotionRequest.odometryData;
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
