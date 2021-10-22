/**
 * @file DribbleEngine.cpp
 *
 * This file implements a module that provides a dribble generator.
 *
 * @author Philip Reichenberg
 */

#include "DribbleEngine.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(DribbleEngine, motionControl);

void DribbleEngine::update(DribbleGenerator& dribbleGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:DribbleEngine:dribbleInWalkKick");
  dribbleGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(false /* TODO */, lastPhase);
    const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryData.inverse() * motionRequest.odometryData;

    const float timeToReachBall = std::max(0.f, std::max(motionRequest.ballEstimate.position.x() / theWalkingEngineOutput.maxSpeed.translation.x(), motionRequest.ballEstimate.position.x() / theWalkingEngineOutput.maxSpeed.translation.x()) - minBallDistanceForVelocity);
    const Vector2f ballSCS = scsCognition * BallPhysics::propagateBallPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity, timeToReachBall, theBallSpecification.friction);
    const Vector2f perceivedBallPosition = scsCognition * motionRequest.ballEstimate.position;
    const Angle directionSCS = Angle::normalize(scsCognition.rotation + motionRequest.targetDirection);

    const Pose3f swingInSupport3D = (isLeftPhase ? theRobotModel.soleRight.inverse() * theRobotModel.soleLeft : theRobotModel.soleLeft.inverse() * theRobotModel.soleRight);
    const Pose2f swingInSupport(swingInSupport3D.rotation.getZAngle(), swingInSupport3D.translation.head<2>());
    const Pose2f swingSCS = hipOffset * swingInSupport;

    Pose2f dribbleStep;

    // check forward kick that is faster to execute
    Legs::Leg kickLeg = ballSCS.y() > 0.f ? Legs::left : Legs::right;

    float power = calcKickPower(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, directionSCS, motionRequest.kickPower, motionRequest.turnKickAllowed);
    bool isInPositionForKick = theWalkKickGenerator.canStart(WalkKickVariant(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, WalkKicks::Type::forward, kickLeg, power, directionSCS), lastPhase, motionRequest.directionPrecision, motionRequest.alignPrecisely, motionRequest.preStepAllowed, motionRequest.turnKickAllowed, 0.f);

    // check forward kick that is slower but executable on the next step
    if(!isInPositionForKick)
    {
      kickLeg = ballSCS.y() < 0.f ? Legs::left : Legs::right;
      power = calcKickPower(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, directionSCS, motionRequest.kickPower, motionRequest.turnKickAllowed);
      isInPositionForKick = theWalkKickGenerator.canStart(WalkKickVariant(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, WalkKicks::Type::forward, kickLeg, power, directionSCS), lastPhase, motionRequest.directionPrecision, motionRequest.alignPrecisely, motionRequest.preStepAllowed, motionRequest.turnKickAllowed, 0.f);
    }

    isInPositionForKick &= motionRequest.ballTimeWhenLastSeen >= theMotionInfo.lastKickTimestamp;
    // velocity.norm() does not need to be transformed to another coordinate system.
    isInPositionForKick &= motionRequest.ballEstimate.velocity.norm() < 50.f;

    if(isInPositionForKick)
    {
      auto kickPhase = theWalkKickGenerator.createPhase(WalkKickVariant(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, WalkKicks::Type::forward, kickLeg, power, directionSCS), lastPhase, motionRequest.directionPrecision, false, 0.f);
      if(kickPhase != nullptr)
      {
        kickPhase->kickType = kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
        return kickPhase;
      }
      else
        OUTPUT_ERROR("Creating Kick Phase of Type " << TypeRegistry::getEnumName(motionRequest.kickType) << " in WalkToBallAndKickEngine returned an empty kick!");
    }
    MotionRequest::ObstacleAvoidance obstacleAvoidanceSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    return theWalkToBallGenerator.createPhase(calcBasePose(ballSCS, directionSCS, lastSign, motionRequest.turnKickAllowed), ballSCS, perceivedBallPosition.norm(), obstacleAvoidanceSCS, motionRequest.walkSpeed, lastPhase);
  };
}

Pose2f DribbleEngine::calcBasePose(const Vector2f& ballInSCS, Angle directionInSCS, float& sign, const bool turnKickAllowed) const
{
  // Determine the foot which should be behind the ball depending on whether the robot is left or right of the line from the ball to the target.
  // -1 = dribble with left foot, 1 = dribble with right foot
  const Angle angleDiff = Angle::normalize(directionInSCS - ballInSCS.angle());
  if(std::abs(angleDiff) > redecideSignThreshold)
    sign = static_cast<float>(sgnPos(angleDiff));

  Pose2f kickPose(directionInSCS, ballInSCS);
  KickInfo::KickType forwardKick = sign > 0.f ? KickInfo::walkForwardsRight : KickInfo::walkForwardsLeft;
  KickInfo::KickType turnKick = forwardKick == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;
  const Angle useDirection = turnKickAllowed ? directionInSCS : 0_deg;
  const float factor = Rangef::ZeroOneRange().limit(useDirection / -theKickInfo[turnKick].rotationOffset);
  kickPose.rotate(theKickInfo[forwardKick].rotationOffset * (1.f - factor) + theKickInfo[turnKick].rotationOffset * factor);
  kickPose.translate(theKickInfo[forwardKick].ballOffset * (1.f - factor) + theKickInfo[turnKick].ballOffset * factor);
  return kickPose;
}

float DribbleEngine::calcKickPower(const KickInfo::KickType kickType, const Angle directionInSCS, const float kickPower, const bool turnKickAllowed)
{
  Rangef useKickRange = theKickInfo[kickType].range;
  const float useTargetRange = kickRange.max * kickPower + kickRange.min * kickPower;
  const bool isLeft = kickType == KickInfo::KickType::walkForwardsLeft;
  const Angle useMaxKickAngle = -theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].rotationOffset;
  const Rangea angleClip(!isLeft ? 0_deg : useMaxKickAngle, !isLeft ? useMaxKickAngle : 0_deg);
  const Angle useDirection = turnKickAllowed ? directionInSCS : 0_deg;
  const float interpolation = Rangef::ZeroOneRange().limit(angleClip.limit(useDirection) / -theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].rotationOffset);
  useKickRange.min = (1 - interpolation) * theKickInfo[kickType].range.min + interpolation * theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].range.min;
  useKickRange.max = (1 - interpolation) * theKickInfo[kickType].range.max + interpolation * theKickInfo[!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight].range.max;
  return std::min(1.f, std::max(0.f, useTargetRange - useKickRange.min) / (useKickRange.max - useKickRange.min));
}
