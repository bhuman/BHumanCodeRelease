/**
 * @file DribbleEngine.cpp
 *
 * This file implements a module that provides a dribble generator.
 *
 * @author Philip Reichenberg
 */

#include "DribbleEngine.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Debugging/Annotation.h"
#include "Math/BHMath.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/Transformation.h"

MAKE_MODULE(DribbleEngine);

using namespace Motion::Transformation;

void DribbleEngine::update(DribbleGenerator& dribbleGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:DribbleEngine:dribbleInWalkKick");
  dribbleGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f(0.f, motionRequest.ballEstimate.position));
    const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theRobotDimensions, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);

    const Rangef ttrbRange(1.f, 2.f);
    const float timeToReachBall = ttrbRange.limit(std::max(std::abs(motionRequest.ballEstimate.position.x()) / theWalkingEngineOutput.maxSpeed.translation.x(), std::abs(motionRequest.ballEstimate.position.y()) / theWalkingEngineOutput.maxSpeed.translation.y()));

    const Vector2f perceivedBallPosition = scsCognition * motionRequest.ballEstimate.position;
    const float ballVelocityFactor = mapToRange(perceivedBallPosition.norm(), ballVelocityInterpolationRange.min, ballVelocityInterpolationRange.max, 0.f, 1.f);
    const float ballVelocity = motionRequest.ballEstimate.velocity.norm();
    const float useVelocityLength = (1.f - ballVelocityFactor) * std::min(ballVelocity, minBallVelocityCloseRange) + ballVelocityFactor * ballVelocity;
    const Vector2f ballSCS = scsCognition * BallPhysics::propagateBallPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity.normalized(useVelocityLength), timeToReachBall, theBallSpecification.friction);

    const Angle directionSCS = Angle::normalize(scsCognition.rotation + motionRequest.targetDirection);

    if(theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen) < 3000.f)
    {
      // check forward kick that is faster to execute
      Legs::Leg kickLeg = ballSCS.y() > 0.f ? Legs::left : Legs::right;

      WalkKickVariant walkKickVariant(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, WalkKicks::Type::forward, kickLeg, motionRequest.alignPrecisely, motionRequest.kickLength, directionSCS, false);
      bool isInPositionForKick = theWalkKickGenerator.canStart(walkKickVariant, lastPhase, motionRequest.directionPrecision, motionRequest.preStepType, motionRequest.turnKickAllowed);

      // check forward kick that is slower but executable on the next step
      if(!isInPositionForKick)
      {
        kickLeg = ballSCS.y() < 0.f ? Legs::left : Legs::right;
        walkKickVariant = WalkKickVariant(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, WalkKicks::Type::forward, kickLeg, motionRequest.alignPrecisely, motionRequest.kickLength, directionSCS, false);
        isInPositionForKick = theWalkKickGenerator.canStart(walkKickVariant, lastPhase, motionRequest.directionPrecision, motionRequest.preStepType, motionRequest.turnKickAllowed);
      }

      isInPositionForKick &= motionRequest.ballTimeWhenLastSeen >= theMotionInfo.lastKickTimestamp;
      // velocity.norm() does not need to be transformed to another coordinate system.
      isInPositionForKick &= motionRequest.ballEstimate.velocity.norm() < maxBallVelocity;

      if(isInPositionForKick)
      {
        const KickInfo::KickType kickType = kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
        auto kickPhase = theWalkKickGenerator.createPhase(walkKickVariant, lastPhase, false);
        if(kickPhase)
        {
          kickPhase->kickType = kickType;
          return kickPhase;
        }
        else
          ANNOTATION("DribbleEngine", "Creating Kick Phase of Type " << TypeRegistry::getEnumName(kickType) << " in WalkToBallAndKickEngine returned an empty kick!");
      }
    }
    MotionRequest::ObstacleAvoidance obstacleAvoidanceSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    Vector2f newBallSCS = ballSCS;
    calcInterceptionPosition(motionRequest, newBallSCS, perceivedBallPosition, scsCognition);
    return theWalkToBallGenerator.createPhase(calcBasePose(newBallSCS, directionSCS, lastSign, motionRequest.turnKickAllowed), newBallSCS, theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen), scsCognition, obstacleAvoidanceSCS, motionRequest.walkSpeed, lastPhase);
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

void DribbleEngine::calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                             const Vector2f& ballInSCSNow, const Pose2f& scsCognition)
{
  if(std::abs(ballInSCSNow.angle() - ballSCS.angle()) > 45_deg)   // ball will land behind us, but is currently in front of us
  {
    const Angle velocityVector = scsCognition.rotation + motionRequest.ballEstimate.velocity.angle();
    const Vector2f minDistancePoint = ballInSCSNow.rotated(-velocityVector);
    const bool faraway = std::abs(minDistancePoint.x()) > minBallPositionFrontSide;
    const bool endPositionFar = ballSCS.squaredNorm() > sqr(minBallPositionFuture);
    if((faraway && endPositionFar))
    {
      const float distance = (ballSCS - ballInSCSNow).norm();
      ballSCS = ballInSCSNow + (ballSCS - ballInSCSNow).normalized(std::min(distance, minBallPositionFrontSide));
    }
  }
}
