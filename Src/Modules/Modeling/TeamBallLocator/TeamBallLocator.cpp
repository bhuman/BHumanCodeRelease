/**
* @file TeamBallLocator.cpp
 *
 * Declares a class that provides a ball model that incorporates information
 * from my teammates.
 *
 * @author Tim Laue
 */

#include "TeamBallLocator.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(TeamBallLocator, modeling)


void TeamBallLocator::update(TeamBallModel& teamBallModel)
{
  // Check and adapt size of buffer -----
  if(balls.size() != static_cast<std::size_t>(Global::getSettings().highestValidPlayerNumber + 1))
    balls.resize(Global::getSettings().highestValidPlayerNumber + 1);
  // Add new observations to buffer
  updateBalls();
  // Compute model
  teamBallModel.isValid = computeTeamBallModel(teamBallModel.position, teamBallModel.velocity);
  if(teamBallModel.isValid)
    teamBallModel.timeWhenLastValid = theFrameInfo.time;
}

void TeamBallLocator::updateBalls()
{
  // I have new ball information!
  unsigned i = theRobotInfo.number;
  if(balls[i].size() == 0 || balls[i][0].time != theBallModel.timeWhenLastSeen || balls[i][0].timeWhenDisappeared != theBallModel.timeWhenDisappeared)
  {
    Ball newBall;
    newBall.robotPose           = theRobotPose;
    newBall.pos                 = theBallModel.estimate.position;
    newBall.vel                 = theBallModel.estimate.velocity;
    newBall.time                = theBallModel.timeWhenLastSeen;
    newBall.timeWhenDisappeared = theBallModel.timeWhenDisappeared;
    newBall.valid = true;
    balls[i].push_front(newBall);
  }
  // Observations by teammates:
  for(auto const& teammate : theTeammateData.teammates)
  {
    // teammate is participating in the game and has made a new ball observation
    // in case of a drop-in game, the teammate must be reliable, too
    unsigned n = teammate.number;
    if(teammate.status == Teammate::FULLY_ACTIVE &&
       (balls[n].size() == 0 || balls[n][0].time != teammate.ball.timeWhenLastSeen || balls[n][0].timeWhenDisappeared != teammate.ball.timeWhenDisappeared) &&
       (!Global::getSettings().isDropInGame || theTeammateReliability.states[n] == TeammateReliability::GOOD))
    {
      Ball newBall;
      newBall.robotPose           = teammate.pose;
      newBall.pos                 = teammate.ball.estimate.position;
      newBall.vel                 = teammate.ball.estimate.velocity;
      newBall.time                = teammate.ball.timeWhenLastSeen;
      newBall.timeWhenDisappeared = teammate.ball.timeWhenDisappeared;
      newBall.valid = true;
      balls[teammate.number].push_front(newBall);
    }
  }
  // If any teammate is not FULLY_ACTIVE anymore, invalidate the observations that happened during the time
  // before the status changed:
  for(auto const& teammate : theTeammateData.teammates)
  {
    if(teammate.status != Teammate::FULLY_ACTIVE)
    {
      unsigned n = teammate.number;
      for(Ball& ball : balls[n])
      {
        if(theFrameInfo.getTimeSince(ball.time) <= inactivityInvalidationTimeSpan)
          ball.valid = false;
        else
          break;
      }
    }
  }
}

bool TeamBallLocator::computeTeamBallModel(Vector2f& pos, Vector2f& vel) const
{
  float weightSum = 0.f;
  Vector2f avgPos(0.f, 0.f);
  Vector2f avgVel(0.f, 0.f);
  for(unsigned int robot=0; robot<balls.size(); ++robot)
  {
    if(balls[robot].size() > 2) // We want more than just the initial entry
    {
      size_t i = 0;
      while(i<balls[robot].size())
      {
        if(balls[robot][i].valid)
          break;
        else
          ++i;
      }
      if(i<balls[robot].size())
      {
        const TeamBallLocator::Ball& ball = balls[robot][i];
        float weighting = computeWeighting(ball);
        if(weighting != 0.f)
        {
          Vector2f absPos = ball.robotPose * ball.pos;
          Vector2f absVel = ball.vel;
          absVel.rotate(ball.robotPose.rotation);
          if(ball.time < theFrameInfo.time)
          {
            float t = (theFrameInfo.time - ball.time) / 1000.f;
            BallPhysics::propagateBallPositionAndVelocity(absPos, absVel, t, theFieldDimensions.ballFriction);
          }
          if(theFieldDimensions.isInsideCarpet(absPos))
          {
            avgPos += absPos * weighting;
            avgVel += absVel * weighting;
            weightSum += weighting;
          }
        }
      }
    }
  }
  if(weightSum != 0.f)
  {
    pos = avgPos / weightSum;
    vel = avgVel / weightSum;
  }
  return weightSum != 0.f;
}

float TeamBallLocator::computeWeighting(const TeamBallLocator::Ball& ball) const
{
  if(theFrameInfo.getTimeSince(ball.time) > ballLastSeenTimeout)
    return 0.f;

  float weighting = 1.0f - (1.0f / (1.0f + std::exp(-(theFrameInfo.getTimeSince(ball.time) - (float)ballLastSeenTimeout) / scalingFactorBallSinceLastSeen))); // sigmoid function based on ball_time_since_last_seen
  weighting *= 1.0f - (1.0f / (1.0f + exp(-(theFrameInfo.getTimeSince(ball.timeWhenDisappeared) - ballDisappearedTimeout) / scalingFactorBallDisappeared)));  // sigmoid function based on ball_time_when_disappeared

  const float camHeight = 550.f;
  const float angleOfCamera = std::atan(ball.pos.norm() / camHeight); // angle of camera when looking at the ball
  const float ballPositionWithPositiveDeviation = std::tan(angleOfCamera + 1_deg) * camHeight; // ball position when angle of camera is a bit different in position direction
  const float ballPositionWithNegativeDeviation = std::tan(angleOfCamera - 1_deg) * camHeight; // ball position when angle of camera is a bit different in negative direction
  const float ballDeviation = (ballPositionWithNegativeDeviation - ballPositionWithPositiveDeviation) / 2; // averaged devition of the ball position
  weighting *= 1.0f / ballDeviation;

  return std::abs(weighting);
}
