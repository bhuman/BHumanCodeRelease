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

  // Configure dynamic values for diagonal kick
  ASSERT(forwardReferenceKick.size() == 2);
  ASSERT(sideReferenceKick.size() == 2);
  ASSERT(forwardLongReferenceKick.size() == 2);
  ASSERT(walkKicksList[WalkKicks::forwardSteal].kickKeyFrame.size() == 1);

  struct DummyPhase : MotionPhase
  {
    using MotionPhase::MotionPhase;

    bool isDone(const MotionRequest&) const override { return true; }
    void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
  };

  auto stepTimingRatio = [&](const Vector2f& ballPosition, const int index, const KickInfo::KickType kickType, const WalkKicks::Type walkKick,
                             const Angle direction, const float power, const int kickIndex) -> std::tuple<std::vector<float>, std::vector<Pose2f>>
  {
    // Get ball position
    Pose2f scsCognition;

    // Get complete odometry for kick direction
    Pose2f footPose;
    Pose2f lastExecutedStep;
    Angle convertedOdometryRotation = 0_deg;
    std::vector<Pose2f> originalTargets;
    std::vector<Pose2f> stepTargets;
    std::vector<Vector2f> stepSwingOffsets;
    WalkKickVariant kick;
    kick.kickType = kickType;
    kick.kickLeg = Legs::left;
    kick.direction = direction;
    kick.walkKickType = walkKick;
    kick.power = power;
    kick.kickIndex = kickIndex;

    getNextStepPositions(true, originalTargets, stepTargets, stepSwingOffsets, kick, index,
                         ballPosition, convertedOdometryRotation, footPose);

    std::vector<float> stepRatio;
    calcStepTimingRatios(stepTargets, lastExecutedStep, stepRatio, kick);
    return { stepRatio, stepTargets };
  };

  // 0 degree direction min
  ASSERT(walkKicksList[WalkKicks::forward].kickKeyFrame.size() == 2);
  ASSERT(walkKicksList[WalkKicks::forward].kickKeyFrame[1].keyframes.size() == 3);
  const Vector2f ballForward = -theKickInfo[KickInfo::walkForwardsLeft].ballOffset - Vector2f(0.f, theRobotDimensions.yHipOffset);
  auto [stepRatioForwardMin, stepsForwardMin] = stepTimingRatio(ballForward, 1, KickInfo::walkForwardsLeft, WalkKicks::forward, 0_deg, 0.f, 1);
  ASSERT(stepRatioForwardMin.size() == 3);
  ASSERT(stepsForwardMin.size() == 3);
  forwardReferenceKick[0].ballDistanceAlign = (stepsForwardMin[0].translation - ballForward).norm();
  forwardReferenceKick[0].ballDistanceKick = stepsForwardMin[1].translation.norm();
  forwardReferenceKick[0].kickLength = theKickInfo[KickInfo::walkForwardsLeft].range.min;
  forwardReferenceKick[0].executionTime = { stepRatioForwardMin[0], stepRatioForwardMin[1] };
  ASSERT(forwardReferenceKick[0].ballDistanceAlign > 0);
  ASSERT(forwardReferenceKick[0].ballDistanceKick > 0);
  ASSERT(forwardReferenceKick[0].kickLength > 0);
  ASSERT(forwardReferenceKick[0].executionTime[0] > 0);
  ASSERT(forwardReferenceKick[0].executionTime[1] > 0);
  // 0 degree direction max
  auto [stepRatioForwardMax, stepsForwardMax] = stepTimingRatio(ballForward, 1, KickInfo::walkForwardsLeft, WalkKicks::forward, 0_deg, 1.f, 1);
  ASSERT(stepRatioForwardMax.size() == 3);
  ASSERT(stepsForwardMax.size() == 3);
  forwardReferenceKick[1].ballDistanceAlign = (stepsForwardMax[0].translation - ballForward).norm();
  forwardReferenceKick[1].ballDistanceKick = stepsForwardMax[1].translation.norm();
  forwardReferenceKick[1].kickLength = 0.001f + theKickInfo[KickInfo::walkForwardsLeft].range.max;
  forwardReferenceKick[1].executionTime = { stepRatioForwardMax[0], stepRatioForwardMax[1] };
  ASSERT(forwardReferenceKick[1].ballDistanceAlign > 0);
  ASSERT(forwardReferenceKick[1].ballDistanceKick > 0);
  ASSERT(forwardReferenceKick[1].kickLength > 0);
  ASSERT(forwardReferenceKick[1].executionTime[0] > 0);
  ASSERT(forwardReferenceKick[1].executionTime[1] > 0);
  ASSERT(forwardReferenceKick[0].kickLength != forwardReferenceKick[1].kickLength);

  // 90 degree direction min
  ASSERT(walkKicksList[WalkKicks::sidewardsOuter].kickKeyFrame.size() == 1);
  ASSERT(walkKicksList[WalkKicks::sidewardsOuter].kickKeyFrame[0].keyframes.size() == 3);
  const Vector2f ballSide = -theKickInfo[KickInfo::walkSidewardsLeftFootToLeft].ballOffset - Vector2f(0.f, theRobotDimensions.yHipOffset);
  auto [stepRatioSideMin, stepsSideMin] = stepTimingRatio(ballSide, 0, KickInfo::walkSidewardsLeftFootToLeft, WalkKicks::sidewardsOuter, 90_deg, 0.f, 0);
  ASSERT(stepRatioSideMin.size() == 3);
  ASSERT(stepsSideMin.size() == 3);
  sideReferenceKick[0].ballDistanceAlign = (stepsSideMin[0].translation - ballSide).norm();
  sideReferenceKick[0].ballDistanceKick = stepsSideMin[1].translation.norm();
  sideReferenceKick[0].kickLength = theKickInfo[KickInfo::walkSidewardsLeftFootToLeft].range.min;
  sideReferenceKick[0].executionTime = { stepRatioSideMin[0], stepRatioSideMin[1] };
  ASSERT(sideReferenceKick[0].ballDistanceAlign > 0);
  ASSERT(sideReferenceKick[0].ballDistanceKick > 0);
  ASSERT(sideReferenceKick[0].kickLength > 0);
  ASSERT(sideReferenceKick[0].executionTime[0] > 0);
  ASSERT(sideReferenceKick[0].executionTime[1] > 0);
  // 90 degree direction max
  auto [stepRatioSideMax, stepsSideMax] = stepTimingRatio(ballSide, 0, KickInfo::walkSidewardsLeftFootToLeft, WalkKicks::sidewardsOuter, 90_deg, 1.f, 0);
  sideReferenceKick[1].ballDistanceAlign = (stepsSideMax[0].translation - ballSide).norm();
  sideReferenceKick[1].ballDistanceKick = stepsSideMax[1].translation.norm();
  sideReferenceKick[1].kickLength = 0.001f + theKickInfo[KickInfo::walkSidewardsLeftFootToLeft].range.max;
  ASSERT(stepRatioSideMax.size() == 3);
  ASSERT(stepsSideMax.size() == 3);
  sideReferenceKick[1].executionTime = { stepRatioSideMax[0], stepRatioSideMax[1] };
  ASSERT(sideReferenceKick[1].ballDistanceAlign > 0);
  ASSERT(sideReferenceKick[1].ballDistanceKick > 0);
  ASSERT(sideReferenceKick[1].kickLength > 0);
  ASSERT(sideReferenceKick[1].executionTime[0] > 0);
  ASSERT(sideReferenceKick[1].executionTime[1] > 0);
  ASSERT(sideReferenceKick[1].kickLength != sideReferenceKick[0].kickLength);

  // 0 degree direction min
  ASSERT(walkKicksList[WalkKicks::forwardLong].kickKeyFrame.size() == 2);
  ASSERT(walkKicksList[WalkKicks::forwardLong].kickKeyFrame[0].keyframes.size() == 2);
  const Vector2f ballForwardLong = -theKickInfo[KickInfo::walkForwardsLeftLong].ballOffset - Vector2f(0.f, theRobotDimensions.yHipOffset);
  auto [stepRatioLongMin, stepsLongMin] = stepTimingRatio(ballForwardLong, 1, KickInfo::walkForwardsLeftLong, WalkKicks::forwardLong, 0_deg, 0.f, 1);
  ASSERT(stepRatioLongMin.size() == 3);
  ASSERT(stepsLongMin.size() == 3);
  forwardLongReferenceKick[0].ballDistanceAlign = (stepsLongMin[0].translation - ballForwardLong).norm();
  forwardLongReferenceKick[0].ballDistanceKick = stepsLongMin[1].translation.norm();
  forwardLongReferenceKick[0].kickLength = theKickInfo[KickInfo::walkForwardsLeftLong].range.min;
  forwardLongReferenceKick[0].executionTime = { stepRatioLongMin[0], stepRatioLongMin[1] };
  ASSERT(forwardLongReferenceKick[0].ballDistanceAlign > 0);
  ASSERT(forwardLongReferenceKick[0].ballDistanceKick > 0);
  ASSERT(forwardLongReferenceKick[0].kickLength > 0);
  ASSERT(forwardLongReferenceKick[0].executionTime[0] > 0);
  ASSERT(forwardLongReferenceKick[0].executionTime[1] > 0);
  // 0 degree direction max
  auto [stepRatioLongMax, stepsLongMax] = stepTimingRatio(ballForwardLong, 1, KickInfo::walkForwardsLeftLong, WalkKicks::forwardLong, 0_deg, 1.f, 1);
  ASSERT(stepRatioSideMax.size() == 3);
  ASSERT(stepsLongMax.size() == 3);
  forwardLongReferenceKick[1].ballDistanceAlign = (stepsLongMax[0].translation - ballForwardLong).norm();
  forwardLongReferenceKick[1].ballDistanceKick = stepsLongMax[1].translation.norm();
  forwardLongReferenceKick[1].kickLength = 0.001f + theKickInfo[KickInfo::walkForwardsLeftLong].range.max;
  forwardLongReferenceKick[1].executionTime = { stepRatioLongMax[0], stepRatioLongMax[1] };
  ASSERT(forwardLongReferenceKick[1].ballDistanceAlign > 0);
  ASSERT(forwardLongReferenceKick[1].ballDistanceKick > 0);
  ASSERT(forwardLongReferenceKick[1].kickLength > 0);
  ASSERT(forwardLongReferenceKick[1].executionTime[0] > 0);
  ASSERT(forwardLongReferenceKick[1].executionTime[1] > 0);
  ASSERT(forwardLongReferenceKick[0].kickLength != forwardLongReferenceKick[1].kickLength);

  // 0 degree direction min
  ASSERT(walkKicksList[WalkKicks::forwardAlternative].kickKeyFrame.size() == 2);
  ASSERT(walkKicksList[WalkKicks::forwardAlternative].kickKeyFrame[0].keyframes.size() == 2);
  const Vector2f ballForwardAlternativ = -theKickInfo[KickInfo::walkForwardsLeftAlternative].ballOffset - Vector2f(0.f, theRobotDimensions.yHipOffset);
  auto [stepRatioAlternativMin, stepsAlternativMin] = stepTimingRatio(ballForwardAlternativ, 1, KickInfo::walkForwardsLeftAlternative, WalkKicks::forwardAlternative, 0_deg, 0.f, 1);
  ASSERT(stepRatioAlternativMin.size() == 3);
  ASSERT(stepsAlternativMin.size() == 3);
  forwardAlternativReferenceKick[0].ballDistanceAlign = (stepsAlternativMin[0].translation - ballForwardAlternativ).norm();
  forwardAlternativReferenceKick[0].ballDistanceKick = stepsAlternativMin[1].translation.norm();
  forwardAlternativReferenceKick[0].kickLength = theKickInfo[KickInfo::walkForwardsLeftAlternative].range.min;
  forwardAlternativReferenceKick[0].executionTime = { stepRatioAlternativMin[0], stepRatioAlternativMin[1] };
  ASSERT(forwardAlternativReferenceKick[0].ballDistanceAlign > 0);
  ASSERT(forwardAlternativReferenceKick[0].ballDistanceKick > 0);
  ASSERT(forwardAlternativReferenceKick[0].kickLength > 0);
  ASSERT(forwardAlternativReferenceKick[0].executionTime[0] > 0);
  ASSERT(forwardAlternativReferenceKick[0].executionTime[1] > 0);
  // 0 degree direction max
  auto [stepRatioAlternativMax, stepsAlternativMax] = stepTimingRatio(ballForwardAlternativ, 1, KickInfo::walkForwardsLeftAlternative, WalkKicks::forwardAlternative, 0_deg, 1.f, 1);
  ASSERT(stepRatioSideMax.size() == 3);
  ASSERT(stepsAlternativMax.size() == 3);
  forwardAlternativReferenceKick[1].ballDistanceAlign = (stepsAlternativMax[0].translation - ballForwardAlternativ).norm();
  forwardAlternativReferenceKick[1].ballDistanceKick = stepsAlternativMax[1].translation.norm();
  forwardAlternativReferenceKick[1].kickLength = 0.001f + theKickInfo[KickInfo::walkForwardsLeftAlternative].range.max;
  forwardAlternativReferenceKick[1].executionTime = { stepRatioAlternativMax[0], stepRatioAlternativMax[1] };
  ASSERT(forwardAlternativReferenceKick[1].ballDistanceAlign > 0);
  ASSERT(forwardAlternativReferenceKick[1].ballDistanceKick > 0);
  ASSERT(forwardAlternativReferenceKick[1].kickLength > 0);
  ASSERT(forwardAlternativReferenceKick[1].executionTime[0] > 0);
  ASSERT(forwardAlternativReferenceKick[1].executionTime[1] > 0);
  ASSERT(forwardAlternativReferenceKick[0].kickLength != forwardAlternativReferenceKick[1].kickLength);

  // Init the sole rectangles
  soleForward = Rangef(-theFootOffset.backward, theFootOffset.forward);
  leftSoleSide = Rangef(-theFootOffset.leftFoot.right, theFootOffset.leftFoot.left);
  rightSoleSide = Rangef(-theFootOffset.rightFoot.right, theFootOffset.rightFoot.left);

  soleForwardWithBall = Rangef(-theFootOffset.backward, theFootOffset.forward + theBallSpecification.radius);
  leftSoleSideWithBall = Rangef(-theFootOffset.leftFoot.right - theBallSpecification.radius, theFootOffset.leftFoot.left + theBallSpecification.radius);
  rightSoleSideWithBall = Rangef(-theFootOffset.rightFoot.right - theBallSpecification.radius, theFootOffset.rightFoot.left + theBallSpecification.radius);
}

void WalkKickEngine::update(WalkKickGenerator& walkKickGenerator)
{
  walkKickGenerator.drawStep = [this](const Pose2f& step)
  {
    draw(step);
  };

  walkKickGenerator.getVShapeWalkStep = [this](const bool isLeftPhase, const Angle direction, const float stealXShift)
  {
    return getVShapeWalkStep(isLeftPhase, direction, stealXShift);
  };

  walkKickGenerator.canStart = [this](WalkKickVariant& walkKickVariant, const MotionPhase& lastPhase, const Rangea& precisionRange,
                                      const PreStepType preStepType, const bool turnKickAllowed)
  {
    WalkKickVariant kick = walkKickVariant;

    // No InWalkKicks after standing
    if(lastPhase.type != MotionPhase::walk)
      return false;

    // After an InWalkKick at least one normal walk phase must follow
    if(theWalkGenerator.wasLastPhaseInWalkKick(lastPhase))
      return false;

    // Diagonal kicks are allowed when we just wanto to hit the ball
    kick.diagonalKickInfo.diagonalKickState = allowDiagonalKicks && kick.precision == KickPrecision::justHitTheBall && kick.walkKickType != WalkKicks::forwardSteal ? WalkKickVariant::DiagonalKickState::allowed : WalkKickVariant::DiagonalKickState::none;

    // Get last step size
    Pose2f leftPose(0_deg, 0.f, 0.f);
    Pose2f rightPose(0_deg, 0.f, 0.f);
    Pose2f lastExecutedStep(0_deg, 0.f, 0.f);
    std::tie(leftPose, rightPose, lastExecutedStep) = theWalkGenerator.getStartOffsetOfNextWalkPhase(lastPhase);
    const Pose2f lastStepChange = theWalkGenerator.getLastStepChange(lastPhase);
    const Pose2f maxOfLastStepExecution(std::max(std::abs(lastExecutedStep.rotation), std::abs(lastStepChange.rotation)), std::max(std::abs(lastExecutedStep.translation.x()), std::abs(lastStepChange.translation.x())), std::max(std::abs(lastExecutedStep.translation.y()), std::abs(lastStepChange.translation.y())));

    // Is left the swing foot?
    bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());

    // Get position of kicking foot, as if the feet would return first back to zero
    const Pose3f nonKickFoot = kick.kickLeg != Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
    Pose3f kickFootPosition;
    if(kick.walkKickType == WalkKicks::forwardSteal)
      kickFootPosition = !isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight;
    else
      kickFootPosition = kick.kickLeg == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;

    Angle rotationOffset = 0_deg;
    if(kick.walkKickType == WalkKicks::forwardSteal)
      rotationOffset = !isLeftPhase ? -forwardStealVFeetAngle : forwardStealVFeetAngle;

    const bool preStepIsNext = kick.kickLeg != (isLeftPhase ? Legs::left : Legs::right);

    // Calculate ball model relative to the kicking foot
    const Pose3f supportInTorso3D = (theTorsoMatrix * kickFootPosition).rotateZ(rotationOffset);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f scsCognition = supportInTorso.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
    kick.ballEstimationTime = kick.ballEstimationTime >= 0.f ? kick.ballEstimationTime : ((preStepIsNext ? stepDuration : 0.f) + timeToHitBall) / 1000.f;
    const Vector2f ballModel = scsCognition * BallPhysics::propagateBallPosition(theMotionRequest.ballEstimate.position, theMotionRequest.ballEstimate.velocity, kick.ballEstimationTime, theBallSpecification.friction);

    // Can the kick be executed based on the swing foot and the kick type?
    bool canKick = preStepIsNext && preStepType != PreStepType::notAllowed && !((kick.walkKickType == WalkKicks::forwardLong && walkKickVariant.precision == KickPrecision::precise));
    if(!preStepIsNext && preStepType != PreStepType::forced && ((kick.walkKickType == WalkKicks::forward && std::abs(walkKickVariant.direction) < forwardPreStepSkipMaxKickAngle) || walkKickVariant.walkKickType == WalkKicks::forwardLong || walkKickVariant.walkKickType == WalkKicks::forwardAlternative))
      canKick = true;
    if(kick.walkKickType == WalkKicks::sidewardsOuter || kick.walkKickType == WalkKicks::forwardSteal)
      canKick = !preStepIsNext && maxOfLastStepExecution.translation.x() < sidewardsOuterForwardRestriction;

    bool isStepSizeTooLarge = false;
    bool stabilizationNecessary = false;
    handleLargeStepSize(isStepSizeTooLarge, canKick, stabilizationNecessary, isLeftPhase, kick, lastExecutedStep, lastStepChange, &lastPhase);
    kick.delayParams.forceDelay = stabilizationNecessary;

    bool canKickAfterCheck = canKick && findBestKickExecution(canKick, precisionRange, ballModel, kick,
                                                              lastStepChange, kick.precision, turnKickAllowed, preStepIsNext);
    if(canKickAfterCheck)
    {
      WalkKickStep dummyKickStep;
      setWalkKickStepInitial(dummyKickStep, kick, lastPhase);
      canKickAfterCheck &= canKickStepSize(dummyKickStep, kick, isLeftPhase, lastPhase);
    }

    if(!canKickAfterCheck && kick.diagonalKickInfo.diagonalKickState == WalkKickVariant::allowed)
    {
      kick = walkKickVariant;
      kick.ballEstimationTime = kick.ballEstimationTime >= 0.f ? kick.ballEstimationTime : ((preStepIsNext ? stepDuration : 0.f) + timeToHitBall) / 1000.f;
      kick.delayParams.forceDelay = stabilizationNecessary;

      canKick = findBestKickExecutionDiagonal(precisionRange, kick,
                                              lastStepChange, kick.precision, preStepIsNext);
      if(canKick)
      {
        WalkKickStep dummyKickStep;
        setWalkKickStepInitial(dummyKickStep, kick, lastPhase);
        canKick &= canKickStepSize(dummyKickStep, kick, isLeftPhase, lastPhase);
      }
      if(canKick && kick.diagonalKickInfo.diagonalKickState == WalkKickVariant::set)
        ANNOTATION("WalkKickEngine", "diagonal: " << TypeRegistry::getEnumName(kick.kickType) << " " << kick.direction.toDegrees());
    }
    else
      canKick &= canKickAfterCheck;

    if(kick.walkKickType == WalkKicks::forwardSteal)
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

    WalkKickStep kickStep;
    ASSERT(kickStep.currentKickVariant.has_value());
    WalkKickVariant& kickVariant = *kickStep.currentKickVariant;
    setWalkKickStepInitial(kickStep, walkKickVariant, lastPhase);
    // TODO the kick can still be aborted in the pre step for some reason... I think. This results in a false sound
    if(playKickSounds && playSound)
      SystemCall::say((kickTypesSpeech[kickVariant.walkKickType] + (kickVariant.kickLeg == Legs::left ? " left" : " right")).c_str());

    return createPhaseWithNextPhase(lastPhase, kickStep);
  };

  walkKickGenerator.dynamicWalkKickStepUpdate = [this](WalkKickStep& kickStep, const float timeSinceStepStarted)
  {
    if(kickStep.isReplayWalkRequest)
      return false;
    ASSERT(kickStep.currentKickVariant.has_value());
    WalkKickVariant& kickVariant = *kickStep.currentKickVariant;
    // kick index is already increased. Normally you would check index 0 and 1.
    if((kickVariant.kickIndex != 2 && kickStep.currentKick != WalkKicks::forwardLong) || // all none forwardLong only adjust in the kick
       (kickVariant.kickIndex > 2 && kickStep.currentKick == WalkKicks::forwardLong) || // forwardLong also adjusts in the prestep
       (kickStep.currentKick == WalkKicks::sidewardsOuter || kickStep.currentKick == WalkKicks::forwardSteal))
      return false;
    // skip the first frame and only update for the second and third frame
    const float threshold = kickVariant.kickIndex == 1 && kickStep.currentKick == WalkKicks::forwardLong ? 5.5f : 3.5f;
    if(timeSinceStepStarted < Constants::motionCycleTime * 1.5f || timeSinceStepStarted > Constants::motionCycleTime * threshold)
      return false;
    WalkKickStep updatedWalkKickStep;
    kickVariant.kickIndex--;
    if(createWalkKickStep(kickStep, updatedWalkKickStep, 0))  // null pointer, because some code needs to use a MotionPhase, which is skipped if the phase is a null pointer
    {
      kickStep = updatedWalkKickStep;
      kickVariant.kickIndex++;
      return true;
    }
    kickVariant.kickIndex++;
    return false;
  };

  MODIFY("module:WalkKickEngine:enableDrawings", enableDrawings);
  DECLARE_DEBUG_DRAWING3D("module:WalkKickEngine:nextSupport", "robot");
  DECLARE_DEBUG_DRAWING3D("module:WalkKickEngine:diagonalKick:contactPoint", "robot");
  draw(theWalkStepData.stepTarget);
}

void WalkKickEngine::setWalkKickStepInitial(WalkKickStep& walkKickStep, const WalkKickVariant& currentKick, const MotionPhase& lastPhase)
{
  ASSERT(walkKickStep.currentKickVariant.has_value());
  WalkKickVariant& kickVariant = *walkKickStep.currentKickVariant;
  walkKickStep.currentKickVariant = currentKick;
  walkKickStep.lastStepInfo.nextIsLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f());
  walkKickStep.lastStepInfo.wasLastPhaseLeftPhase = theWalkGenerator.wasLastPhaseLeftPhase(lastPhase);
  const bool isLeftPhase = walkKickStep.lastStepInfo.nextIsLeftPhase;
  kickVariant.kickIndex = (currentKick.walkKickType == WalkKicks::forward || currentKick.walkKickType == WalkKicks::forwardLong || currentKick.walkKickType == WalkKicks::forwardAlternative) &&
                          ((currentKick.kickLeg == Legs::left && isLeftPhase) || (currentKick.kickLeg == Legs::right && !isLeftPhase)) ? 1 : 0;
  walkKickStep.lastStepInfo.wasLastPhaseInWalkKick = theWalkGenerator.wasLastPhaseInWalkKick(lastPhase);

  std::tie(walkKickStep.lastStepInfo.leftStartOffset, walkKickStep.lastStepInfo.rightStartOffset, walkKickStep.lastStepInfo.lastExecutedStep) = theWalkGenerator.getStartOffsetOfNextWalkPhase(lastPhase);
}

void WalkKickEngine::handleLargeStepSize(bool& isStepSizeTooLarge, bool& canKick, bool& stabilizationNecessary, const bool isLeftPhase, const WalkKickVariant& walkKickVariant, const Pose2f& lastExecutedStep, const Pose2f& lastStepChange, const MotionPhase* lastPhase)
{
  const bool preStepIsNext = walkKickVariant.kickLeg != (isLeftPhase ? Legs::left : Legs::right);
  const bool mirrorYCheck = (preStepIsNext && isLeftPhase) || (!preStepIsNext && !isLeftPhase);
  const Rangef sideThresholdStabilization(mirrorYCheck ? -restrictedYTranslationOfPreStep.y() : -2000.f, mirrorYCheck ? 2000.f : restrictedYTranslationOfPreStep.y());
  const Rangef sideThresholdCritical(mirrorYCheck ? -restrictedYTranslationOfPreStepWithStabilization : -2000.f, mirrorYCheck ? 2000.f : restrictedYTranslationOfPreStepWithStabilization);

  isStepSizeTooLarge = !sideThresholdCritical.isInside(lastStepChange.translation.y()) || !sideThresholdCritical.isInside(lastExecutedStep.translation.y());
  stabilizationNecessary = !sideThresholdStabilization.isInside(lastStepChange.translation.y()) || !sideThresholdStabilization.isInside(lastExecutedStep.translation.y());

  if(walkKickVariant.walkKickType == WalkKicks::forward)
  {
    const Rangef forwardRange(walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : 0_deg, walkKickVariant.kickLeg != Legs::left ? -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset : 0_deg);
    const Angle confAngle = forwardRange.limit(walkKickVariant.direction);
    const float interpolation = Rangef::ZeroOneRange().limit(confAngle / (walkKickVariant.kickLeg == Legs::left ? -theKickInfo[KickInfo::walkTurnLeftFootToRight].rotationOffset : -theKickInfo[KickInfo::walkTurnRightFootToLeft].rotationOffset) / 0.5f);
    isStepSizeTooLarge |= std::abs(lastStepChange.translation.x()) > turnKickPreviousMaxXStepSize * (1.f - interpolation) + restrictedYTranslationOfPreStep.x() * interpolation;
  }
  else if(walkKickVariant.walkKickType == WalkKicks::turnOut)
    isStepSizeTooLarge |= std::abs(lastStepChange.translation.x()) > restrictedYTranslationOfPreStep.x();

  if(walkKickVariant.precision == KickPrecision::precise && walkKickVariant.walkKickType == WalkKicks::turnOut)  // force the robot to stop before the kick
    isStepSizeTooLarge |= std::abs(lastStepChange.translation.x()) > forwardTurnPreciseMaxStepSize;

  // A too large side step was previously executed, but a stabilization is not possible -> set critical to true
  isStepSizeTooLarge |= stabilizationNecessary && lastPhase && (theLibDemo.isDemoActive || // side step was too large based on old threshold
                        !((walkKickVariant.delayParams.kickIndex == -1 && theWalkGenerator.isWalkDelayPossible(*lastPhase, stabilizationWalkDelay.max, Pose2f(0.f, 0.f, isLeftPhase ? 1.f : 1.f), true)) || // no delay planned, but one would be possible
                          (walkKickVariant.delayParams.kickIndex >= 0 && walkKickVariant.delayParams.delay > stabilizationWalkDelay.min))); // delay planned which is long enough

  // was last step size too big?
  // this check is needed to ensure that the ball is hit
  if(walkKickVariant.precision != KickPrecision::justHitTheBall && isStepSizeTooLarge)
    for(WalkKicks::Type kickType : walkKicksWithRestrictedStart)
      if(kickType == walkKickVariant.walkKickType)
        canKick = false;
}

bool WalkKickEngine::canKickStepSize(const WalkKickStep& walkKickStep, WalkKickVariant& kick, const bool isLeftPhase, const MotionPhase& lastPhase)
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
  getNextStepPositionsSpecialHandling(walkKickStep.lastStepInfo, kick, theOdometryDataPreview, kickIndex, originalTargets, stepTargets, stepSwingOffset, lastExecutedStep);

  // Abort if needed
  Rangef maxClipX;
  Rangef maxClipY;
  Rangea maxClipRot;
  getClipRanges(maxClipX, maxClipY, maxClipRot, kick, kickIndex, kick.precision == KickPrecision::justHitTheBall);
  if(!canExecute(originalTargets, stepTargets, maxClipX, maxClipY, maxClipRot,
                 kick.kickLeg != Legs::left, kick, isLeftPhase, &lastPhase, lastExecutedStep)
     || forwardStealAbortCondition(kick, stepTargets, kickIndex))
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
                                                   const bool isPreStep)
{
  if(isPreStep || (theKickInfo[walkKickVariant.kickType].walkKickType != WalkKicks::forward &&
                   theKickInfo[walkKickVariant.kickType].walkKickType != WalkKicks::forwardLong &&
                   theKickInfo[walkKickVariant.kickType].walkKickType != WalkKicks::forwardAlternative))
    return false;

  walkKickVariant.diagonalKickInfo.diagonalKickState = WalkKickVariant::set;
  const bool isLeftPhase = walkKickVariant.kickLeg == Legs::left;
  const Pose3f& kickFootPosition = isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight;
  const Pose2f kickSole2D(kickFootPosition.rotation.getZAngle(), kickFootPosition.translation.head<2>());
  const Rangef& kickSoleSide = isLeftPhase ? leftSoleSide : rightSoleSide;
  const Rangef& kickSoleSideWithBall = isLeftPhase ? leftSoleSideWithBall : rightSoleSideWithBall;
  const Pose2f scsCognition = kickSole2D.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
  const Vector2f ballPosition = scsCognition * BallPhysics::propagateBallPosition(theMotionRequest.ballEstimate.position, theMotionRequest.ballEstimate.velocity, walkKickVariant.ballEstimationTime, theBallSpecification.friction);

  // If the ball is inside the foot, we do not execute a diagonal kick
  if(soleForward.isInside(ballPosition.x()) && kickSoleSide.isInside(ballPosition.y()))
    return false;
  if(ballPosition.x() < 0.f)
    return false;

  auto xInterpolation = [&](const float ballPositionX)
  {
    const float minForward = soleForward.max - theBallSpecification.radius;
    const float maxForward = soleForwardWithBall.max;
    if(ballPositionX < minForward)
      return ballPositionX;
    return mapToRange(ballPositionX, minForward, maxForward, minForward, soleForward.max);
  };

  auto yInterpolation = [&](const float ballPositionY, const Rangef& kickSoleSideWithBall, const Rangef& kickSoleSide)
  {
    // ensure 0 is the middle point
    if(ballPositionY > 0)
      return mapToRange(ballPositionY, 0.f, kickSoleSideWithBall.max, 0.f, kickSoleSide.max);
    else
      return mapToRange(ballPositionY, kickSoleSideWithBall.min, 0.f, kickSoleSide.min, 0.f);
  };

  Pose2f zeroStepSCSCognition;
  const Vector2f zeroStepBallPosition = getZeroBallPosition(isLeftPhase, zeroStepSCSCognition, walkKickVariant.ballEstimationTime);
  const Vector2f contactPoint(xInterpolation(ballPosition.x()), yInterpolation(ballPosition.y(), kickSoleSideWithBall, kickSoleSide));
  const Vector2f contactPointZeroStep = zeroStepSCSCognition * kickSole2D * contactPoint;
  const Angle zeroStepKickDirection = (zeroStepBallPosition - contactPointZeroStep).angle();

  COMPLEX_DRAWING3D("module:WalkKickEngine:diagonalKick:contactPoint")
  {
    const Vector2f contactPointInRobotPose = kickSole2D * contactPoint;
    POINT3D("module:WalkKickEngine:diagonalKick:contactPoint", contactPointInRobotPose.x(), contactPointInRobotPose.y(), -theTorsoMatrix.translation.z(), 10, ColorRGBA::orange);
    const Angle kickAngleInRobotPose = -zeroStepSCSCognition.rotation + zeroStepKickDirection;
    const Vector2f kickLine = Vector2f::polar(300.f, kickAngleInRobotPose);
    LINE3D("module:WalkKickEngine:diagonalKick:contactPoint", contactPointInRobotPose.x(), contactPointInRobotPose.y(), -theTorsoMatrix.translation.z(), contactPointInRobotPose.x() + kickLine.x(), contactPointInRobotPose.y() + kickLine.y(), -theTorsoMatrix.translation.z(), 10, ColorRGBA::orange);
  }

  // Check if ball x and y is ok
  // Check if kick direction is fine
  const Rangea kickDirectionThreshold(walkKickVariant.direction + precisionRange.min, walkKickVariant.direction + precisionRange.max);
  if(!kickDirectionThreshold.isInside(zeroStepKickDirection))
    return false;

  walkKickVariant.direction = zeroStepKickDirection;
  walkKickVariant.kickInterpolation = 0.f;
  walkKickVariant.diagonalKickInfo.contactPoint = contactPointZeroStep;
  DeviationValues dvForward = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);

  if(walkKickVariant.walkKickType == WalkKicks::forward)
  {
    WalkKickVariant sideVariant = walkKickVariant;
    sideVariant.kickType = walkKickVariant.kickType == KickInfo::walkForwardsLeft ? KickInfo::walkSidewardsLeftFootToLeft : KickInfo::walkSidewardsRightFootToRight;
    const DeviationValues dvSide = getForwardTurnKickDeviation(walkKickVariant, lastExecutedStep, precision, isPreStep);
    const float diagonalFactor = mapToRange(std::abs(zeroStepKickDirection), 0.f, static_cast<float>(90_deg), 0.f, 1.f);
    dvForward.maxXDeviation = Rangef(dvSide.maxXDeviation.min * diagonalFactor + dvForward.maxXDeviation.min * (1.f - diagonalFactor),
                                     dvSide.maxXDeviation.max * diagonalFactor + dvForward.maxXDeviation.max * (1.f - diagonalFactor));
    dvForward.maxYDeviation = Rangef(dvSide.maxYDeviation.min * diagonalFactor + dvForward.maxYDeviation.min * (1.f - diagonalFactor),
                                     dvSide.maxYDeviation.max * diagonalFactor + dvForward.maxYDeviation.max * (1.f - diagonalFactor));
    dvForward.maxAngleDeviation = dvSide.maxAngleDeviation * diagonalFactor + dvForward.maxAngleDeviation * (1.f - diagonalFactor);
    const float sidePower = KickLengthConverter::kickLengthToPower(sideVariant.kickType, walkKickVariant.length, walkKickVariant.direction, theKickInfo);
    walkKickVariant.power = sidePower * diagonalFactor + walkKickVariant.power * (1.f - diagonalFactor);
    walkKickVariant.kickInterpolation = diagonalFactor;
    // TODO just check the distance?
  }

  // the robot can stand further away from the ball, if the previous step was bigger.
  float xRangeMaxOffset = 0.f;
  if(walkKickVariant.walkKickType == WalkKicks::forwardLong) /* || walkKickVariant.walkKickType == WalkKicks::forwardAlternative) Deactivate for alternative version */
  {
    if(std::abs(ballPosition.y()) > 50.f)
      return false;
    Rangef maxAdditionalXRange(0.f, maxForward.max / 2.f);
    xRangeMaxOffset += maxAdditionalXRange.limit(lastExecutedStep.translation.x() + maxForwardAcceleration);
  }
  const Rangef xRange(0.f, dvForward.maxXDeviation.max + xRangeMaxOffset);
  return xRange.min < ballPosition.x() &&  // ball is far enough away from kicking foot
         xRange.max > zeroStepBallPosition.x() && // ball is probably reachable
         dvForward.maxYDeviation.isInside(ballPosition.y()); // ball is not too far to the side
}

void WalkKickEngine::getClipRanges(Rangef& maxClipX, Rangef& maxClipY, Rangea& maxClipRot,
                                   const WalkKickVariant& kick, const int kickIndex,
                                   const bool isJustHitTheBall)
{
  if(kick.walkKickType != WalkKicks::forwardLong && kick.diagonalKickInfo.diagonalKickState == WalkKickVariant::DiagonalKickState::set)
  {
    maxClipX = Rangef(-40.f, 40.f); // TODO eval
    maxClipY = Rangef(-40.f, 40.f); // TODO eval
    maxClipRot = Rangea(-0.1_deg, 0.1_deg); // Rot is not clipped anyway
    return;
  }
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
}

void WalkKickEngine::clipKickDirectionWithPrecision(Angle& direction, const WalkKickVariant& walkKickVariant, const Rangea& precisionRange)
{
  const Rangea precision(walkKickVariant.direction + precisionRange.min, walkKickVariant.direction + precisionRange.max);
  direction = precision.limit(-theKickInfo[walkKickVariant.kickType].rotationOffset);
}

void WalkKickEngine::clipStepTarget(Pose2f& original, Pose2f& newPose,
                                    const bool isLeftPhase, const KickKeyframePart keyframePart,
                                    const WalkKickVariant& kick, const Pose2f& lastExecutedStep,
                                    const float maxSideStep)
{
  // No modification of the max speed. This should have happened in applyAllClipping. Only check if this actually happened!
  VERIFY(maxSideStep >= 0.f);
  if(kick.diagonalKickInfo.diagonalKickState != WalkKickVariant::set)
  {
    Angle currentRot = newPose.rotation;
    clipRotation(currentRot, isLeftPhase);
    ASSERT(currentRot == newPose.rotation);
  }
  ASSERT(maxForward.isInside(newPose.translation.x()));
  const Rangef maxSideStepClip((isLeftPhase ? maxSide.min : -maxSideStep) - 0.1f, (isLeftPhase ? maxSideStep : -maxSide.min) + 0.1f);
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

void WalkKickEngine::getNextStepPositionsSpecialHandling(const WalkKickStep::LastStepInfo& lastStepInfo, const WalkKickVariant& kick, const OdometryData& odometryData,
                                                         const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                                         std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep)
{
  if(kick.walkKickType == WalkKicks::forward && kick.diagonalKickInfo.diagonalKickState != WalkKickVariant::set)
  {
    std::vector<Pose2f> forwardStepTargets, forwardOriginalTargets, turnStepTargets, turnOriginalTargets;
    std::vector<Vector2f> forwardSwingOffset, turnSwingOffset;
    WalkKickVariant forwardKick = kick;
    WalkKickVariant turnKick = kick;
    turnKick.walkKickType = WalkKicks::Type::turnOut;

    getNextStepPositionsWithClipping(lastStepInfo, forwardKick, odometryData, index, forwardOriginalTargets, forwardStepTargets, forwardSwingOffset, lastExecutedStep);
    getNextStepPositionsWithClipping(lastStepInfo, turnKick, odometryData, index, turnOriginalTargets, turnStepTargets, turnSwingOffset, lastExecutedStep);
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
    getNextStepPositionsWithClipping(lastStepInfo, kick, odometryData, index, originalTargets, stepTargets, stepSwingOffsets, lastExecutedStep);
}

void WalkKickEngine::applyAllClipping(const WalkKickVariant& kick, const bool isLeftPhase,
                                      const int index, std::vector<Pose2f>& stepTargets,
                                      Pose2f& lastExecutedStep, const bool isDiagonalKick)
{
  std::vector<Vector2f> translationPolygon;
  Vector2f backRight(-theWalkingEngineOutput.maxPossibleBackwardStepSize, -theWalkingEngineOutput.maxPossibleStepSize.translation.y());
  Vector2f frontLeft(theWalkingEngineOutput.maxPossibleStepSize.translation);
  const Rangef maxSideStepClip(
    isLeftPhase ? maxSide.min : std::min(-0.01f, -walkKicksList[kick.walkKickType].kickKeyFrame[index].maxSideStep),
    isLeftPhase ? std::max(0.01f, walkKicksList[kick.walkKickType].kickKeyFrame[index].maxSideStep) : -maxSide.min);

  // Set min step target

  if(!isDiagonalKick && walkKicksList[kick.walkKickType].kickKeyFrame[index].clipMaxSpeed)
    frontLeft.x() = std::max(maxForwardAcceleration, std::min(frontLeft.x(), lastExecutedStep.translation.x() + maxForwardAcceleration));
  if(kick.precision == KickPrecision::justHitTheBall || kick.walkKickType == WalkKicks::forwardLong)
    frontLeft.x() = std::min(frontLeft.x(), kick.walkKickType == WalkKicks::forwardLong ? (theLibDemo.isDemoActive || theLibDemo.isOneVsOneDemoActive ? forwardLongMaxStepSizeDemo : forwardLongMaxStepSize) : (theLibDemo.isDemoActive || theLibDemo.isOneVsOneDemoActive ? forwardLongMaxStepSizeDemo : justHitTheBallMaxStepSize));

  // clip max allowed step size
  // different than from WalkingEngine, because kicks are allowed to be different!
  frontLeft.x() = maxForward.limit(frontLeft.x());
  backRight.x() = maxForward.limit(backRight.x());
  frontLeft.y() = maxSideStepClip.limit(frontLeft.y());
  backRight.y() = maxSideStepClip.limit(backRight.y());

  // TODO do not use lastExecutedStep, but the measured position of both feet.
  // This would need a conversion function from theWalkGeneratorOutput, because of the arm compensation and torso shift
  // The walkingEngine already has a conversion like this, but only internally
  auto applyHoldClip = [&](const std::size_t i)
  {
    ASSERT(i < stepTargets.size());
    if(!isDiagonalKick && walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].holdXTranslationWhenMovingBackward)
      stepTargets[i].translation.x() = std::max(stepTargets[i].translation.x(), i == 0 ? -lastExecutedStep.translation.x() : stepTargets[i - 1].translation.x());

    if(!isDiagonalKick && walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].holdYTranslationWhenFeetTogether)
    {
      if(isLeftPhase)
        stepTargets[i].translation.y() = std::min(stepTargets[i].translation.y(), stepTargets[stepTargets.size() - 1].translation.y());
      else
        stepTargets[i].translation.y() = std::max(stepTargets[i].translation.y(), stepTargets[stepTargets.size() - 1].translation.y());
    }
  };

  theWalkGenerator.generateTranslationPolygon(isLeftPhase, 0, Pose2f(1.f, 1.f, 1.f), translationPolygon, backRight, frontLeft, true, true);
  for(std::size_t i = 0; i < stepTargets.size(); i++)
  {
    // Make sure the point (0,0) is always part of the polygon
    ASSERT(Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), Vector2f(0.f, 0.f)));

    if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), stepTargets[i].translation))
    {
      Vector2f p1(0.f, 0.f);
      VERIFY((Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(i == 0 ? Vector2f(0.f, 0.f) : stepTargets[i - 1].translation,
                                                              (i == 0 ? stepTargets[i].translation : stepTargets[i].translation - stepTargets[i - 1].translation).normalized(0.1f)), p1, false)));
      stepTargets[i].translation = p1 * 0.99f; // slightly reduce step because of floating point errors
    }
    clipRotation(stepTargets[i].rotation, isLeftPhase);
    // Apply special clipping afterwards. Those clipping can not be applied with the translation polygon
    // Also it needs to be after all other clipping as the last step keyframe is used here
    applyHoldClip(i);

    // Make sure the endposition is inside the polygon
    // Otherwise the next keyframe could crash
    if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), stepTargets[i].translation))
    {
      Vector2f p1(0.f, 0.f);
      VERIFY((Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f),
                                                              (stepTargets[i].translation).normalized(0.1f)), p1, false)));
      stepTargets[i].translation = p1 * 0.99f; // slightly reduce step because of floating point errors
    }
  }
}

void WalkKickEngine::getNextStepPositionsWithClipping(const WalkKickStep::LastStepInfo& lastStepInfo, const WalkKickVariant& kick, const OdometryData& odometryData,
                                                      const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                                      std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep)
{
  // Get step targets
  const bool isLeftPhase = lastStepInfo.nextIsLeftPhase;
  const Pose2f& leftPose = lastStepInfo.leftStartOffset;
  const Pose2f& rightPose = lastStepInfo.rightStartOffset;
  lastExecutedStep = lastStepInfo.lastExecutedStep;

  if(kick.diagonalKickInfo.diagonalKickState == WalkKickVariant::set)
  {
    Pose2f scsCognition;
    const Vector2f ballPosition = getZeroBallPosition(isLeftPhase, scsCognition, kick.ballEstimationTime);
    const Vector2f& kickSole = scsCognition * (isLeftPhase ? theRobotModel.soleLeft : theRobotModel.soleRight).translation.head<2>();
    getNextStepPositionsDiagonal(originalTargets, stepTargets, stepSwingOffsets, kick, ballPosition, lastExecutedStep.rotation, kickSole);

    applyAllClipping(kick, isLeftPhase, index, stepTargets, lastExecutedStep, true);
    for(size_t i = 0; i < stepTargets.size(); i++)
    {
      ASSERT(maxForward.isInside(stepTargets[i].translation.x()));
      const Rangef maxSideStepClip((isLeftPhase ? maxSide.min : -walkKicksList[kick.walkKickType].kickKeyFrame[index].maxSideStep) - 0.1f, (isLeftPhase ? walkKicksList[kick.walkKickType].kickKeyFrame[index].maxSideStep : -maxSide.min) + 0.1f);
      ASSERT(maxSideStepClip.isInside(stepTargets[i].translation.y()));
    }
    return;
  }

  // Get ball position
  Pose2f scsCognition;
  const Vector2f ballPosition = getZeroBallPosition(isLeftPhase, scsCognition, kick.ballEstimationTime);

  // Get complete odometry for kick direction
  Angle convertedOdometryRotation = scsCognition.rotation - (theOdometryDataPreview.rotation - odometryData.rotation);
  getNextStepPositions(isLeftPhase, originalTargets, stepTargets, stepSwingOffsets, kick, index,
                       ballPosition, convertedOdometryRotation, isLeftPhase ? leftPose : rightPose);

  applyAllClipping(kick, isLeftPhase, index, stepTargets, lastExecutedStep);
  for(size_t i = 0; i < stepTargets.size(); i++)
  {
    clipStepTarget(originalTargets[i], stepTargets[i], isLeftPhase,
                   walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i], kick, lastExecutedStep, walkKicksList[kick.walkKickType].kickKeyFrame[index].maxSideStep);
  }
}

void WalkKickEngine::getNextStepPositions(const bool isLeftPhase, std::vector<Pose2f>& original, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                                          const WalkKickVariant& kick, const int index, const Vector2f& ballPosition,
                                          const Angle convertedOdometryRotation, const Pose2f& lastExecutedSupportStep)
{
  if(index >= static_cast<int>(walkKicksList[kick.walkKickType].kickKeyFrame.size()))
    return;

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
    Vector2f stepTargetPositionRelativeToBall = getPowerScaledRPB(kick.walkKickType, index, i, kick.power) + minStepOffset;
    stepTargetPositionRelativeToBall.y() *= sign;

    Vector2f stepTargetPositionRelativeToBallWithOffset = stepTargetPositionRelativeToBall - getPowerScaledOSF(kick.walkKickType, index, i, kick.power);
    Angle waitRotationPosition = walkKicksList[kick.walkKickType].kickKeyFrame[index].keyframes[i].directionRatio * (kick.direction + convertedOdometryRotation);

    clipRotation(waitRotationPosition, isLeftPhase); // Use the rotation, that will be used at the end after the clipping. Otherwise the target position can be off

    stepTargetPositionRelativeToBall.rotate(waitRotationPosition);
    stepTargetPositionRelativeToBallWithOffset.rotate(waitRotationPosition);
    const Pose2f target = Pose2f(waitRotationPosition, ballPosition + stepTargetPositionRelativeToBall); // But save the original planned rotation, so the check if the step is possible still checks for the rotation
    const Pose2f targetWithOffset = Pose2f(waitRotationPosition, ballPosition + stepTargetPositionRelativeToBallWithOffset);

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

void WalkKickEngine::getNextStepPositionsDiagonal(std::vector<Pose2f>& original, std::vector<Pose2f>& poses,
                                                  std::vector<Vector2f>& stepSwingOffsets, const WalkKickVariant& kick,
                                                  const Vector2f& ballPosition, const Angle lastStepRotation, const Vector2f& kickSole)
{
  std::vector<Pose2f> stepTargets; // Step targets for walk phase
  std::vector<Vector2f> stepTargetsWithoutOffset; // Step targets for walk phase
  constexpr unsigned maxNumOfStepKeyFrames = 2;
  for(std::size_t i = 0; i < maxNumOfStepKeyFrames; i++)
  {
    // i == 0 -> distance to the ball
    // i == 1 -> how much does the kick foot must move for the kick?
    // TODO min of 0 for alignment is a bad idea. Use distance of contact point to ball
    float distance = getPowerScaleDiagonalKick(kick.length, kick.kickInterpolation, kick.walkKickType, i == 0);

    const float minDistance = (ballPosition - kick.diagonalKickInfo.contactPoint).norm() - (i == 0 ? theBallSpecification.radius - 10.f : 0.f); // 10mm as save distance
    distance = i == 0 ? std::min(distance, minDistance) : std::max(distance, minDistance);

    const Vector2f moveingVector = Vector2f::polar(distance, Angle::normalize(kick.direction));

    const Vector2f point = kickSole + moveingVector;
    stepTargets.emplace_back(-lastStepRotation, point.x(), point.y());
    stepTargetsWithoutOffset.push_back(Vector2f(0.f, 0.f));
  }
  poses = stepTargets;
  stepSwingOffsets = stepTargetsWithoutOffset;

  poses.push_back(poses.back());
  stepSwingOffsets.push_back(stepSwingOffsets.back());
  original = poses;
  ASSERT(poses.size() == stepSwingOffsets.size());
}

bool WalkKickEngine::canExecute(const std::vector<Pose2f>& originalPoses, const std::vector<Pose2f>& clippedPoses, const Rangef& maxClipBeforeAbortX,
                                const Rangef& maxClipBeforeAbortY, const Rangea& maxClipBeforeAbortRot, const bool mirror, const WalkKickVariant& kick,
                                const bool isLeftPhase, const MotionPhase* lastPhase, const Pose2f& lastExecutedStep)
{
  // Note: lastPhase can be a nullptr
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

    // Check if normal kicks not be possible in the prestep
    if(kick.precision != KickPrecision::justHitTheBall && kick.kickLeg != (isLeftPhase ? Legs::left : Legs::right))
    {
      for(WalkKicks::Type kickType : walkKicksWithRestrictedStart)
        if(kickType == kick.walkKickType)
        {
          bool isStepTooLarge;
          bool isStabilizationNecessary;
          bool canKick;
          handleLargeStepSize(isStepTooLarge, canKick, isStabilizationNecessary, isLeftPhase, kick, lastExecutedStep, Pose2f(), lastPhase);
          reachable &= !isStepTooLarge;
        }
    }
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

bool WalkKickEngine::abortKick(const MotionPhase& lastPhase, const WalkKickStep& walkKickStep)
{
  // There was a false support switch in the current InWalkKick. Abort it.
  if(walkKickStep.lastStepInfo.wasLastPhaseInWalkKick && walkKickStep.lastStepInfo.wasLastPhaseLeftPhase == walkKickStep.lastStepInfo.nextIsLeftPhase)
  {
    ANNOTATION("WalkKickEngine", "Wrong support foot switch");
    return true;
  }

  ASSERT(walkKickStep.currentKickVariant.has_value());
  const WalkKickVariant& kickVariant = *walkKickStep.currentKickVariant;

  return lastPhase.type != MotionPhase::walk || // No InWalkKicks after standing
         kickVariant.kickIndex >= static_cast<int>(walkKicksList[kickVariant.walkKickType].kickKeyFrame.size()) || // InWalkKick is over
         forwardStealAbortCondition(kickVariant, std::vector<Pose2f>(), kickVariant.kickIndex) ||
         (kickVariant.walkKickType == WalkKicks::forwardSteal && theFrameInfo.getTimeSince(theMotionRequest.ballTimeWhenLastSeen) > timeSinceBallLastSeenThreshold); // Ball was not seen for a too long time
}

bool WalkKickEngine::createWalkKickStep(const WalkKickStep& walkKickStep, WalkKickStep& nextWalkKickStep, const MotionPhase* lastPhase)
{
  ASSERT(walkKickStep.currentKickVariant.has_value());
  const WalkKickStep::LastStepInfo& lastStepInfo = walkKickStep.lastStepInfo;
  const WalkKickVariant& currentKick = *walkKickStep.currentKickVariant;
  const int kickIndex = currentKick.kickIndex;

  std::vector<Pose2f> originalTargets, stepTargets;
  std::vector<Vector2f> stepSwingOffset;
  Pose2f lastExecutedStep;
  getNextStepPositionsSpecialHandling(walkKickStep.lastStepInfo, currentKick, odometryAtStart, kickIndex, originalTargets, stepTargets, stepSwingOffset, lastExecutedStep);

  // Abort if needed
  Rangef maxClipX;
  Rangef maxClipY;
  Rangea maxClipRot;
  getClipRanges(maxClipX, maxClipY, maxClipRot, currentKick, kickIndex, currentKick.precision == KickPrecision::justHitTheBall);

  bool isLeftPhase = lastStepInfo.nextIsLeftPhase;
  if(!canExecute(originalTargets, stepTargets, maxClipX, maxClipY, maxClipRot,
                 currentKick.kickLeg != Legs::left, currentKick, isLeftPhase, lastPhase, lastExecutedStep))
  {
    if(kickIndex > 0 && !lastStepInfo.wasLastPhaseInWalkKick && (currentKick.walkKickType == WalkKicks::forward || currentKick.walkKickType == WalkKicks::forwardLong))
      ANNOTATION("WalkKickEngine", "Kick aborted, which already started");
    return false;
  }

  std::vector<float> stepRatio;
  calcStepTimingRatios(stepTargets, lastExecutedStep, stepRatio, currentKick);

  // Set parameters for the step targets
  const Legs::Leg swingLeg = isLeftPhase ? Legs::left : Legs::right;
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
      const Vector2f useSpeedUpSwingFactorForward = Vector2f((1.f - currentKick.power) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionXScaling.min +
                                                             currentKick.power * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionXScaling.max,
                                                             (1.f - currentKick.power) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionYScaling.min +
                                                             currentKick.power * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionYScaling.max);
      const Vector2f useSpeedUpSwingFactorTurnOut = Vector2f((1.f - currentKick.power) * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionXScaling.min +
                                                             currentKick.power * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionXScaling.max,
                                                             (1.f - currentKick.power) * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionYScaling.min +
                                                             currentKick.power * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionYScaling.max);
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
      // TODO diagonal kick
      keyframe.speedUpSwing = Vector2f((1.f - currentKick.power) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionXScaling.min +
                                       currentKick.power * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionXScaling.max,
                                       (1.f - currentKick.power) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionYScaling.min +
                                       currentKick.power * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].speedUpSwingPositionYScaling.max);
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

  WalkKickStep& kickStep = nextWalkKickStep;
  kickStep.keyframe = stepKeyframes;
  // TODO diagonal kick
  kickStep.increaseSwingHeightFactor = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].increaseSwingHeightFactor;
  WalkKicks::Type jointOffsetsKick = currentKick.walkKickType;
  // TODO diagonal kick
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
  // TODO diagonal kicks
  kickStep.currentKick = currentKick.walkKickType;
  kickStep.useSlowSupportFootHeightAfterKickInterpolation = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useSlowSupportFootHeightAfterKickInterpolation;
  if(currentKick.walkKickType == WalkKicks::forward)
    kickStep.useSlowSwingFootHeightInterpolation = (1.f - currentKick.kickInterpolation) * walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useSlowSwingFootHeightInterpolation +
                                                   currentKick.kickInterpolation * walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].useSlowSwingFootHeightInterpolation;
  else
    kickStep.useSlowSwingFootHeightInterpolation = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useSlowSwingFootHeightInterpolation;
  kickStep.useLastKeyframeForSupportFootXTranslation = walkKicksList[currentKick.walkKickType].kickKeyFrame[kickIndex].useLastKeyframeForSupportFootXTranslation;

  kickStep.usedWalkDelay = currentKick.delayParams.kickIndex == kickIndex && currentKick.delayParams.delay > 0.f;

  // Copy info
  kickStep.lastStepInfo = walkKickStep.lastStepInfo;
  for(std::size_t keyframeIndex = 0; keyframeIndex < walkKickStep.keyframe.size(); keyframeIndex++)
    kickStep.keyframe[keyframeIndex].holdXSwingTarget = walkKickStep.keyframe[keyframeIndex].holdXSwingTarget;
  kickStep.currentKickVariant = walkKickStep.currentKickVariant;

  return true;
}

std::unique_ptr<MotionPhase> WalkKickEngine::createPhaseWithNextPhase(const MotionPhase& lastPhase, const WalkKickStep& walkKickStep)
{
  if(abortKick(lastPhase, walkKickStep))
    return std::unique_ptr<MotionPhase>();

  WalkKickStep lastWalkStep = walkKickStep;
  ASSERT(lastWalkStep.currentKickVariant.has_value());
  const WalkKickVariant& lastVariant = *lastWalkStep.currentKickVariant;
  setWalkKickStepInitial(lastWalkStep, lastVariant, lastPhase);
  WalkKickStep kickStep;
  if(!createWalkKickStep(lastWalkStep, kickStep, &lastPhase))
    return std::unique_ptr<MotionPhase>();

  ASSERT(kickStep.currentKickVariant.has_value());
  WalkKickVariant& kickVariant = *kickStep.currentKickVariant;

  // Long and turn kicks shall shall use a delay for more stabilization
  // TODO the kick is potentionally less strong. Needs more tests
  // TODO Move magic numbers into config AFTER testing
  // TODO diagonal kicks
  if(kickVariant.delayParams.kickIndex == -1 && kickVariant.kickIndex == 1 && kickVariant.delayParams.forceDelay &&
     (kickVariant.walkKickType == WalkKicks::forwardLong || kickVariant.walkKickType == WalkKicks::forwardAlternative || kickVariant.walkKickType == WalkKicks::turnOut ||
      (kickVariant.walkKickType == WalkKicks::forward && (std::abs(kickVariant.direction) > 20_deg || std::abs(kickStep.lastStepInfo.lastExecutedStep.translation.y()) > 30.f ||
                                                          std::abs(kickStep.keyframe.back().stepTargetSwing.translation.y()) > 30.f))))
  {
    kickVariant.delayParams.kickIndex = 1;
    kickVariant.delayParams.delay = stabilizationWalkDelay.max;
    ANNOTATION("WalkKickEngine", "Added Walkdelay For JustHitTheBall");
  }

  kickVariant.kickIndex++;
  kickVariant.ballEstimationTime -= stepDuration / 1000.f;

  auto phase = theWalkGenerator.createPhaseWithNextPhase(kickStep,
                                                         lastPhase, std::bind(&WalkKickEngine::createPhaseWithNextPhase, this, std::placeholders::_1, std::placeholders::_2), kickVariant.delayParams.kickIndex == kickVariant.kickIndex - 1 ? kickVariant.delayParams.delay : 0.f);

  phase->kickType = lastPhase.kickType;
  return phase;
}

void WalkKickEngine::calcStepTimingRatios(const std::vector<Pose2f>& stepTargets, const Pose2f& lastExecutedStep, std::vector<float>& stepRatio, const WalkKickVariant& kick)
{
  stepRatio.clear();
  std::vector<float> timings;
  if(kick.diagonalKickInfo.diagonalKickState == WalkKickVariant::DiagonalKickState::set)
  {
    ASSERT(kick.walkKickType == WalkKicks::forward || kick.walkKickType == WalkKicks::forwardLong || kick.walkKickType == WalkKicks::forwardAlternative);
    const std::array<WalkKickLengthPair, 2>& forwardArray = kick.walkKickType == WalkKicks::forward ? forwardReferenceKick :
                                                            (kick.walkKickType == WalkKicks::forwardLong ? forwardLongReferenceKick : forwardAlternativReferenceKick);

    ASSERT(forwardArray[0].executionTime.size() == 2);

    const float forwardRatio = mapToRange(kick.length, forwardArray[0].kickLength, forwardArray[1].kickLength, 0.f, 1.f);
    float executionTimeAlign = mapToRange(forwardRatio, 0.f, 1.f, forwardArray[0].executionTime[0], forwardArray[1].executionTime[0]);
    float executionTimeKick = mapToRange(forwardRatio, 0.f, 1.f, forwardArray[0].executionTime[1], forwardArray[1].executionTime[1]);

    if(kick.walkKickType == WalkKicks::forward)
    {
      ASSERT(sideReferenceKick[0].executionTime.size() == 2);

      const float sideRatio = mapToRange(kick.length, sideReferenceKick[0].kickLength, sideReferenceKick[1].kickLength, 0.f, 1.f);
      const float executionTimeAlignSide = mapToRange(sideRatio, 0.f, 1.f, sideReferenceKick[0].executionTime[0], sideReferenceKick[1].executionTime[0]);
      const float executionTimeKickSide = mapToRange(sideRatio, 0.f, 1.f, sideReferenceKick[0].executionTime[1], sideReferenceKick[1].executionTime[1]);

      executionTimeAlign = kick.kickInterpolation * executionTimeAlignSide + executionTimeAlign * (1.f - kick.kickInterpolation);
      executionTimeKick = kick.kickInterpolation * executionTimeKickSide + executionTimeKick * (1.f - kick.kickInterpolation);
    }

    const float thirdStepTime = std::max(0.0001f, 1.f - (executionTimeAlign + executionTimeKick));

    ASSERT(executionTimeAlign >= 0.f);
    ASSERT(executionTimeKick > 0.f);
    ASSERT(thirdStepTime > 0.f);
    ASSERT(Approx::isEqual(executionTimeAlign + executionTimeKick + thirdStepTime, 1.f, 0.01f));

    timings.push_back(executionTimeAlign);
    timings.push_back(executionTimeKick);
    timings.push_back(thirdStepTime);
  }
  else
  {
    // Calc ratios for the step targets
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
      const int kickIndex = kick.kickIndex;
      if(kick.walkKickType == WalkKicks::forward)
      {
        float forwardKickTimeWeighted = (walkKicksList[kick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio ? walkKicksList[kick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio : time) * (1.f - kick.kickInterpolation);
        float turnKickTimeWeighted = (walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio ? walkKicksList[WalkKicks::turnOut].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio : time) * kick.kickInterpolation;
        timings.emplace_back(std::max(0.0001f, forwardKickTimeWeighted + turnKickTimeWeighted));
      }
      // default case, use default value
      else if(walkKicksList[kick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].useDefaultReachPositionRatio)
        timings.emplace_back(walkKicksList[kick.walkKickType].kickKeyFrame[kickIndex].keyframes[i].reachPositionRatio);
      else
        timings.emplace_back(time > 0 ? time : 0.0001f);
    }
  }
  // Set ratios for all step targets
  float sum = 0.f;
  for(const auto& n : timings)
    sum += n;
  for(size_t i = 0; i < timings.size(); i++)
    stepRatio.emplace_back(timings[i] / sum);
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

float WalkKickEngine::getPowerScaleDiagonalKick(const float kickLength, const float ratio, const WalkKicks::Type type, const bool aligning)
{
  ASSERT(type == WalkKicks::forward || type == WalkKicks::forwardLong || type == WalkKicks::forwardAlternative);
  const auto forwardArray = type == WalkKicks::forward ? forwardReferenceKick :
                            (type == WalkKicks::forwardLong ? forwardLongReferenceKick : forwardAlternativReferenceKick);
  const float forwardRatio = mapToRange(kickLength, forwardArray[0].kickLength, forwardArray[1].kickLength, 0.f, 1.f);

  const float distanceMin = aligning ? forwardArray[0].ballDistanceAlign : forwardArray[0].ballDistanceKick;
  const float distanceMax = aligning ? forwardArray[1].ballDistanceAlign : forwardArray[1].ballDistanceKick;
  const float distanceForward = mapToRange(forwardRatio, 0.f, 1.f, distanceMin, distanceMax);
  if(type != WalkKicks::forward)
    return distanceForward;

  // Real diagonal kick => interpolate between side and forward

  const float sideRatio = mapToRange(kickLength, sideReferenceKick[0].kickLength, sideReferenceKick[1].kickLength, 0.f, 1.f);
  const float sideDistanceMin = aligning ? sideReferenceKick[0].ballDistanceAlign : sideReferenceKick[0].ballDistanceKick;
  const float sideDistanceMax = aligning ? sideReferenceKick[1].ballDistanceAlign : sideReferenceKick[1].ballDistanceKick;
  const float sideDistanceForward = mapToRange(sideRatio, 0.f, 1.f, sideDistanceMin, sideDistanceMax);

  return distanceForward * (1.f - ratio) + sideDistanceForward * ratio;

  // TODO ratio applying should be a sin function?
}

Vector2f WalkKickEngine::getZeroBallPosition(const bool isLeftPhase, Pose2f& scsCognition, const float ballTime)
{
  const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
  const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
  const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
  scsCognition = hipOffset * hipOffset * supportInTorso.inverse() * theOdometryDataPreview.inverse() * theMotionRequest.odometryData;
  return scsCognition * BallPhysics::propagateBallPosition(theMotionRequest.ballEstimate.position, theMotionRequest.ballEstimate.velocity, ballTime, theBallSpecification.friction);
}

Pose2f WalkKickEngine::getVShapeWalkStep(const bool isLeftPhase, const Angle direction, const float stealXShift)
{
  Pose2f scsCognition;
  Vector2f ball = getZeroBallPosition(isLeftPhase, scsCognition, 0.f);
  Vector2f offset = forwardStealWaitingKickPose + Vector2f(stealXShift, 0.f);
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
