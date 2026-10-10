/**
 * @file WalkToBallAndKickEngine.cpp
 *
 * This file implements a module that provides a walk to ball and kick generator.
 *
 * @author Arne Hasselbring
 */

#include "WalkToBallAndKickEngine.h"
#include "Debugging/Annotation.h"
#include "Platform/BHAssert.h"
#include "Streaming/Global.h"
#include "Framework/Settings.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/KickLengthConverter.h"

MAKE_MODULE(WalkToBallAndKickEngine);

void WalkToBallAndKickEngine::update(WalkToBallAndKickGenerator& walkToBallAndKickGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:WalkToBallAndKickEngine:forwardTurnInterpolation");
  DECLARE_DEBUG_RESPONSE("module:WalkToBallAndKickEngine:UseMirroredForwardKick");

  walkToBallAndKickGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const Pose2f scsCognition = theOdometryDataPreview.inverse() * motionRequest.odometryData;

    const Rangef ttrbRange(1.f, 2.f);
    const float timeToReachBall = ttrbRange.limit(std::max(std::abs(motionRequest.ballEstimate.position.x()) / theWalkingEngineOutput.maxSpeed.translation.x(), std::abs(motionRequest.ballEstimate.position.y()) / theWalkingEngineOutput.maxSpeed.translation.y()));

    const Vector2f perceivedBallPosition = scsCognition * motionRequest.ballEstimate.position;
    const float ballVelocityFactor = mapToRange(perceivedBallPosition.norm(), ballVelocityInterpolationRange.min, ballVelocityInterpolationRange.max, 0.f, 1.f);
    const float ballVelocity = motionRequest.ballEstimate.velocity.norm();
    const float useVelocityLength = (1.f - ballVelocityFactor) * std::min(ballVelocity, minBallVelocityCloseRange) + ballVelocityFactor * ballVelocity;
    const Vector2f ballPosition = scsCognition * BallPhysics::propagateBallPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity.normalized(useVelocityLength), timeToReachBall, theBallSpecification);

    const Angle targetDirection = Angle::normalize(scsCognition.rotation + motionRequest.targetDirection);

    Pose2f kickPose = Pose2f(targetDirection, ballPosition);

    kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset);
    kickPose.rotation = Angle::normalize(kickPose.rotation);
    kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset);

    MotionRequest::ObstacleAvoidance obstacleAvoidanceInSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceInSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceInSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    Vector2f newBallPosition = ballPosition;
    if(calcInterceptionPosition(motionRequest, newBallPosition, perceivedBallPosition, scsCognition))
    {
      kickPose = Pose2f(targetDirection, newBallPosition);
      kickPose.rotate(theKickInfo[motionRequest.kickType].rotationOffset);
      kickPose.rotation = Angle::normalize(kickPose.rotation);
      kickPose.translate(theKickInfo[motionRequest.kickType].ballOffset);
    }

    return theWalkToBallGenerator.createPhase(kickPose, newBallPosition, theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen), scsCognition, obstacleAvoidanceInSCS, motionRequest.walkSpeed, lastPhase);
  };
}

bool WalkToBallAndKickEngine::calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                                       const Vector2f& ballInSCSNow, const Pose2f& scsCognition)
{
  if(std::abs(ballInSCSNow.angle() - ballSCS.angle()) > 45_deg)  // ball will land behind us, but is currently in front of us
  {
    const Angle velocityVector = scsCognition.rotation + motionRequest.ballEstimate.velocity.angle();
    const Vector2f minDistancePoint = ballInSCSNow.rotated(-velocityVector);
    const bool faraway = std::abs(minDistancePoint.x()) > minBallPositionFrontSide;
    const bool endPositionFar = ballSCS.squaredNorm() > sqr(minBallPositionFuture);
    if((faraway && endPositionFar))
    {
      const float distance = (ballSCS - ballInSCSNow).norm();
      ballSCS = ballInSCSNow + (ballSCS - ballInSCSNow).normalized(std::min(distance, minBallPositionFrontSide));
      return true;
    }
  }
  return false;
}
