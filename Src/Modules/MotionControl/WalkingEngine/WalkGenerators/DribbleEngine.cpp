/**
 * @file DribbleEngine.cpp
 *
 * This file implements a module that provides a dribble generator.
 *
 * @author Philip Reichenberg
 */

#include "DribbleEngine.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Framework/Settings.h"
#include "Debugging/Annotation.h"
#include "Math/BHMath.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(DribbleEngine);

void DribbleEngine::update(DribbleGenerator& dribbleGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:DribbleEngine:dribbleInWalkKick");
  dribbleGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const Pose2f scsCognition = theOdometryDataPreview.inverse() * motionRequest.odometryData;

    const Rangef ttrbRange(1.f, 2.f);
    const float timeToReachBall = ttrbRange.limit(std::max(std::abs(motionRequest.ballEstimate.position.x()) / theWalkingEngineOutput.maxSpeed.translation.x(), std::abs(motionRequest.ballEstimate.position.y()) / theWalkingEngineOutput.maxSpeed.translation.y()));

    const Vector2f perceivedBallPosition = scsCognition * motionRequest.ballEstimate.position;
    const float ballVelocityFactor = mapToRange(perceivedBallPosition.norm(), ballVelocityInterpolationRange.min, ballVelocityInterpolationRange.max, 0.f, 1.f);
    const float ballVelocity = motionRequest.ballEstimate.velocity.norm();
    const float useVelocityLength = (1.f - ballVelocityFactor) * std::min(ballVelocity, minBallVelocityCloseRange) + ballVelocityFactor * ballVelocity;
    const Vector2f ballSCS = scsCognition * BallPhysics::propagateBallPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity.normalized(useVelocityLength), timeToReachBall, theBallSpecification);

    const Angle directionSCS = Angle::normalize(scsCognition.rotation + motionRequest.targetDirection);
    MotionRequest::ObstacleAvoidance obstacleAvoidanceSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    Vector2f newBallSCS = ballSCS;
    calcInterceptionPosition(motionRequest, newBallSCS, perceivedBallPosition, scsCognition);
    return theWalkToBallGenerator.createPhase(calcBasePose(newBallSCS, directionSCS), newBallSCS, theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen), scsCognition, obstacleAvoidanceSCS, motionRequest.walkSpeed, lastPhase);
  };
}

Pose2f DribbleEngine::calcBasePose(const Vector2f& ballInSCS, Angle directionInSCS) const
{
  Pose2f kickPose(directionInSCS, ballInSCS);
  kickPose.rotate(theKickInfo[KickInfo::rlKickDynamic].rotationOffset);
  kickPose.translate(theKickInfo[KickInfo::rlKickDynamic].ballOffset);
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
