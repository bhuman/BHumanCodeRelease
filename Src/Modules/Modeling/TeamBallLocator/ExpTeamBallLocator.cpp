/**
 * @file ExpTeamBallLocator.cpp
 *
 * Declares a class that provides a ball model that incorporates information
 * from my teammates.
 *
 * @author Tim Laue
 */

#include "ExpTeamBallLocator.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(ExpTeamBallLocator, modeling)

void ExpTeamBallLocator::update(TeamBallModel& teamBallModel)
{
  // Check and adapt size of buffer -----
  if(balls.size() != static_cast<std::size_t>(Settings::highestValidPlayerNumber + 1))
    balls.resize(Settings::highestValidPlayerNumber + 1);
  // Add new observations to buffer
  updateBalls();
  // Compute model
  unsigned timeWhenLastSeen = computeTeamBallModel(teamBallModel.position, teamBallModel.velocity);
  teamBallModel.isValid = timeWhenLastSeen != 0;
  if(teamBallModel.isValid)
  {
    teamBallModel.timeWhenLastValid = theFrameInfo.time;
    teamBallModel.timeWhenLastSeen = timeWhenLastSeen;
  }
}

void ExpTeamBallLocator::updateBalls()
{
  // I have new ball information!
  unsigned i = theRobotInfo.number;
  if(balls[i].size() == 0 || balls[i][0].time != theBallModel.timeWhenLastSeen)
  {
    Ball newBall;
    newBall.robotPose           = theRobotPose;
    newBall.pos                 = theBallModel.estimate.position;
    newBall.vel                 = theBallModel.estimate.velocity;
    newBall.time                = theBallModel.timeWhenLastSeen;
    newBall.valid = true;
    balls[i].push_front(newBall);
  }
  // Observations by teammates:
  for(auto const& teammate : theTeammateData.teammates)
  {
    // teammate is participating in the game and has made a new ball observation
    // in case of a drop-in game, the teammate must be reliable, too
    unsigned n = teammate.number;
    if(teammate.status == Teammate::PLAYING &&
       (balls[n].size() == 0 || balls[n][0].time != teammate.ball.timeWhenLastSeen) &&
       teammate.ball.lastPerception != Vector2f::Zero() &&
       teammate.ball.estimate.position != Vector2f::Zero() &&
       (!Global::getSettings().isDropInGame || theTeammateReliability.states[n] == TeammateReliability::GOOD) &&
       !ballIsNearOtherTeammate(teammate))
    {
      Ball newBall;
      newBall.robotPose           = teammate.pose;
      newBall.pos                 = teammate.ball.estimate.position;
      newBall.vel                 = teammate.ball.estimate.velocity;
      newBall.time                = teammate.ball.timeWhenLastSeen;
      newBall.valid = true;
      balls[teammate.number].push_front(newBall);
    }
  }
  // If any teammate is not PLAYING anymore, invalidate the observations that happened during the time
  // before the status changed:
  for(auto const& teammate : theTeammateData.teammates)
  {
    if(teammate.status != Teammate::PLAYING)
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

bool ExpTeamBallLocator::ballIsNearOtherTeammate(const Teammate& teammate)
{
  Vector2f teammateGlobalBallPos = teammate.pose * teammate.ball.estimate.position;
  for(auto const& other : theTeammateData.teammates)
  {
    if(other.number == teammate.number)
      continue;
    if((other.pose.translation - teammateGlobalBallPos).norm() < robotBanRadius)
      return true;
  }
  if((theRobotPose.translation - teammateGlobalBallPos).norm() < robotBanRadius)
    return true;
  return false;
}

unsigned ExpTeamBallLocator::computeTeamBallModel(Vector2f& pos, Vector2f& vel) const
{
  unsigned timeWhenLastSeen = 0;
  float weightSum = 0.f;
  Vector2f avgPos(0.f, 0.f);
  Vector2f avgVel(0.f, 0.f);
  for(unsigned int robot = 0; robot < balls.size(); ++robot)
  {
    if(balls[robot].size() > 2) // We want more than just the initial entry
    {
      size_t i = 0;
      while(i < balls[robot].size())
      {
        if(balls[robot][i].valid)
          break;
        else
          ++i;
      }
      if(i < balls[robot].size())
      {
        const ExpTeamBallLocator::Ball& ball = balls[robot][i];
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
            if(ball.time > timeWhenLastSeen)
              timeWhenLastSeen = ball.time;
          }
        }
      }
    }
  }
  ASSERT(std::isfinite(weightSum));
  if(weightSum != 0.f)
  {
    pos = avgPos / weightSum;
    vel = avgVel / weightSum;
  }
  return timeWhenLastSeen;
}

float ExpTeamBallLocator::computeWeighting(const ExpTeamBallLocator::Ball& ball) const
{
  if(theFrameInfo.getTimeSince(ball.time) > ballLastSeenTimeout)
    return 0.f;

  float weighting = 1.0f - (1.0f / (1.0f + std::exp(-(theFrameInfo.getTimeSince(ball.time) - (float)ballLastSeenTimeout) / scalingFactorBallSinceLastSeen))); // sigmoid function based on ball_time_since_last_seen

  const float camHeight = 550.f;
  const float angleOfCamera = std::atan(ball.pos.norm() / camHeight); // angle of camera when looking at the ball
  const float ballPositionWithPositiveDeviation = std::tan(angleOfCamera + 1_deg) * camHeight; // ball position when angle of camera is a bit different in position direction
  const float ballPositionWithNegativeDeviation = std::tan(angleOfCamera - 1_deg) * camHeight; // ball position when angle of camera is a bit different in negative direction
  const float ballDeviation = (ballPositionWithNegativeDeviation - ballPositionWithPositiveDeviation) / 2; // averaged deviation of the ball position
  weighting *= 1.0f / ballDeviation;

  return std::abs(weighting);
}
