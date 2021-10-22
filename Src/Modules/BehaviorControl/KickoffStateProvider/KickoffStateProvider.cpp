/**
 * @file KickoffStateProvider.cpp
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#include "KickoffStateProvider.h"
#include "Tools/Settings.h"

MAKE_MODULE(KickoffStateProvider, behaviorControl);

KickoffStateProvider::KickoffStateProvider()
{
  reset();
}

void KickoffStateProvider::update(KickoffState& kickoffState)
{
  if(theGameInfo.state != STATE_PLAYING)
  {
    reset();
    return; // Nothing to do here
  }

  // If certain types of free kicks happen, the ball definitely went out of the center circle
  if(theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN)
  {
    ballHasMoved = true;
    ballWasOutOfCenterCircle = true;
  }

  if(theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
    ballWasOutOfCenterCircle = true;

  const bool robotIsCloseToCenterCircle = theRobotPose.translation.norm() < theFieldDimensions.centerCircleRadius + robotOutOfCenterCircleTolerance;

  // BallWasOutOfCenterCircle
  if(!ballWasOutOfCenterCircle)
  {
    if((robotIsCloseToCenterCircle && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) == 0)
       || (!robotIsCloseToCenterCircle && theTeamBallModel.isValid))
    {
      Vector2f ballPosition = theTeamBallModel.position;
      // If we are inside the center circle, we should prefer the own observation only:
      if(robotIsCloseToCenterCircle)
        ballPosition = theRobotPose * theBallModel.estimate.position;

      if(ballPosition.norm() > theFieldDimensions.centerCircleRadius + theFieldDimensions.fieldLinesWidth * 0.5f + theBallSpecification.radius + ballOutOfCenterCircleTolerance)
        ballOutOfCenterCircleCounter++;
      else if(ballOutOfCenterCircleCounter != 0)
        ballOutOfCenterCircleCounter--;

      if(ballOutOfCenterCircleCounter >= ballOutOfCenterCircleCounterThreshold)
        ballWasOutOfCenterCircle = true;
    }
  }

  // BallHasMoved
  if(!ballHasMoved)
  {
    if(theBallModel.timeWhenLastSeen == theFrameInfo.time &&
       theBallModel.estimate.position.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius - ballHasMovedCloseToRobotThreshold))
    {
      ballHasMoved = true;
    }
    else if(theGameInfo.setPlay != SET_PLAY_PENALTY_KICK &&
            (ballWasOutOfCenterCircle ||
             theExtendedGameInfo.timeSincePlayingStarted > opponentKickoffMaxTime ||
             theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber))
    {
      ballHasMoved = true;
    }
    else
    {
      // If robot is moving, the buffer content is invalid:
      if(theMotionInfo.executedPhase != MotionPhase::stand || theGyroState.deviation.y() > gyroThreshold)
      {
        ballPositions.clear();
      }
      else
      {
        // Ball is seen now, and no ball has been entered for a while
        if(theFrameInfo.time != 0 && theBallModel.timeWhenLastSeen == theFrameInfo.time && theFrameInfo.getTimeSince(lastBallAddedToBuffer) > ballSaveInterval)
        {
          ballPositions.push_front(theBallModel.lastPerception);
          lastBallAddedToBuffer = theFrameInfo.time;
        }
      }

      if(ballPositions.size() >= 6)
      {
        const size_t firstIndex = ballPositions.size() - 1;

        const Vector2f& start = medianOfThree(ballPositions.back(),
                                              ballPositions[firstIndex - 1],
                                              ballPositions[firstIndex - 2]);

        const Vector2f& end = medianOfThree(ballPositions.front(),
                                            ballPositions[1],
                                            ballPositions[2]);

        if((start - end).squaredNorm() > sqr(ballHasMovedTolerance))
          ballHasMoved = true;
      }
    }
  }
  kickoffState.allowedToEnterCenterCircle = allowedToEnterCenterCircle();
  kickoffState.allowedToScore = allowedToScore();
}

void KickoffStateProvider::reset()
{
  ballOutOfCenterCircleCounter = 0;
  ballPositions.clear();
  lastBallAddedToBuffer = 0;
  ballHasMoved = false;
  ballWasOutOfCenterCircle = false;
}

bool KickoffStateProvider::allowedToScore()
{
  if(alwaysAllowToScore && (theRobotPose * theBallModel.estimate.position).x() > -theFieldDimensions.centerCircleRadius)
    return true;
  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    return true;
  if(theGameInfo.state != STATE_PLAYING)  // Should not be relevant, but who knows...
    return false;
  return ballWasOutOfCenterCircle;
}

bool KickoffStateProvider::allowedToEnterCenterCircle()
{
  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    return true;
  if(theGameInfo.state == STATE_READY)  // Should not be relevant, but who knows...
    return true;
  if(theGameInfo.setPlay != SET_PLAY_PENALTY_KICK)
    if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
      return true;
  return ballHasMoved;
}

const Vector2f& KickoffStateProvider::medianOfThree(const Vector2f& a, const Vector2f& b, const Vector2f& c) const
{
  const float aN = a.squaredNorm();
  const float bN = b.squaredNorm();
  const float cN = c.squaredNorm();

  if(aN >= bN)
  {
    if(cN >= aN)
      return a;
    else // c < a
      return bN >= cN ? b : c;
  }
  else // b > a
  {
    if(cN >= bN)
      return b;
    else // c < b
      return aN >= cN ? a : c;
  }
}
