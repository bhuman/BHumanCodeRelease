/**
 * @file InterceptBallInformationProvider.cpp
 *
 * Provides information about intercepting the ball for the Behavior
 *
 * @author Philip Reichenberg
 */

#include "InterceptBallInformationProvider.h"
#include "Math/Geometry.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(InterceptBallInformationProvider);

void InterceptBallInformationProvider::update(InterceptBallInformation& theInterceptBallInformation)
{
  const bool lastIntercept = theInterceptBallInformation.intercept;
  const bool lastWalkOutOfPath = theInterceptBallInformation.walkOutOfBallPath;

  // Easy case, do nothing :)
  if(theEmergencyButtonState.buttonPressed || theGameState.stopped || !theGameState.isPlaying() || theGameState.isPenalized())
  {
    theInterceptBallInformation.intercept = false;
    theInterceptBallInformation.walkOutOfBallPath = false;
    return;
  }

  // Check if ball is rolling towards a pass target that is not us
  for(const Teammate& teammate : theTeamData.teammates)
  {
    if(teammate.theBehaviorStatus.passTarget == -1)
      continue;
    if(teammate.theBehaviorStatus.passTarget != theGameState.playerNumber)
    {
      // Get newest pass
      lastPassToOtherTeammatePlanned = std::max(lastPassToOtherTeammatePlanned, teammate.theFrameInfo.time);
      if(lastPassToOtherTeammatePlanned == teammate.theFrameInfo.time)
      {
        const auto passTarget = std::find_if(theTeamData.teammates.begin(), theTeamData.teammates.end(), [&](const Teammate& t) {return t.number == teammate.theBehaviorStatus.passTarget; });
        if(passTarget == theTeamData.teammates.end())  // fall-back
        {
          lastPassToOtherTeammatePlanned = 0;
          continue;
        }
        // Pass from ball of teammate to actual pass target
        oldTeammateStart = teammate.theRobotPose * teammate.theBallModel.estimate.position;
        passAngle = (passTarget->theRobotPose.translation - oldTeammateStart).angle();
        oldTeammateTarget = passTarget->theRobotPose.translation;
      }
    }
    else
    {
      lastPassToOtherTeammatePlanned = 0;
      break;
    }
  }

  if(theBallModel.estimate.velocity == Vector2f::Zero())
  {
    theInterceptBallInformation.intercept = false;
    theInterceptBallInformation.walkOutOfBallPath = false;
    return;
  }

  theInterceptBallInformation.intercept = theFieldInterceptBall.interceptBall || theFieldInterceptBall.predictedInterceptBall;
  theInterceptBallInformation.walkOutOfBallPath = theFieldBall.isRollingTowardsOpponentGoal;

  // Handle debugging in logs
  if(theFrameInfo.time < lastFrameInfo || lastPassToOtherTeammatePlanned > theFrameInfo.time)
    lastPassToOtherTeammatePlanned = 0;

  // Easy case, obviously we want to walk out of the ball path
  // ... but if we are already on the other side of the ball, there is not need to walk out of it
  // Also if we were not previously intercepting and the ball is close, we do not want to start it
  const Angle ballToUsAngle = (theRobotPose.translation - theFieldBall.recentBallPositionOnField()).angle();
  const Angle ballRollAngle = theBallModel.estimate.velocity.angle() + theRobotPose.rotation;
  const float ballDistance = theFieldBall.recentBallPositionRelative().squaredNorm();
  theInterceptBallInformation.walkOutOfBallPath &= std::abs(Angle::normalize(ballToUsAngle - ballRollAngle)) < goalBallAngleDiff + (lastWalkOutOfPath ? goalBallAngleHysteresis : 0_deg);
  theInterceptBallInformation.intercept &= (lastIntercept || ballDistance > sqr(minBallDistanceForWalkOut)) && !theInterceptBallInformation.walkOutOfBallPath;

  // Easy case, obviously we want to keep intercepting
  if(theInterceptBallInformation.intercept && theFieldBall.isRollingTowardsOwnGoal)
    return;

  const Angle globalBallRollAngle = theBallModel.estimate.velocity.angle() + theRobotPose.rotation;
  // A pass is planned that is NOT towards us -> walk out of ball path
  if(theFrameInfo.getTimeSince(lastPassToOtherTeammatePlanned) < rememberPassTime // Pass communicated is fresh
     && std::abs(passAngle - globalBallRollAngle) < maxBallRollAngleDiff // Ball roll path deviates too much from start. Allow intercepting
     && std::abs((theFieldBall.recentBallPositionOnField() - oldTeammateStart).angle() - passAngle) < maxBallRollAngleDiff // New Ball position matches to planned pass
     && std::abs(globalBallRollAngle) < 90_deg && !theFieldBall.isRollingTowardsOwnGoal) // ball is not rolling backwards and risks rolling into our own goal
  {
    theInterceptBallInformation.intercept = false;
    theInterceptBallInformation.walkOutOfBallPath = true;
    return;
  }
  else if(theFrameInfo.getTimeSince(lastPassToOtherTeammatePlanned) > rememberPassTime / 4)
    lastPassToOtherTeammatePlanned = 0;

  // Ball rolls globally > x direction -> check if there is a better teammate in range
  // TODO
}
