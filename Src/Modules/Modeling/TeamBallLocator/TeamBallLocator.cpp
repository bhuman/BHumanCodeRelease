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
  if(balls.size() != static_cast<std::size_t>(Settings::highestValidPlayerNumber + 1))
    balls.resize(Settings::highestValidPlayerNumber + 1);
  // Add new observations to buffer (except for phases during which everything is deleted)
  if(!checkForResetByGameSituation())
    updateInternalBallBuffers();
  // Reset lists and variables:
  ballsAvailableForTeamBall.clear();
  teamBallModel.isValid = false;
  // Try to compute a team ball
  findAvailableBalls(false);
  // If we play a mixed team game and did not find any available balls so far, try to use the
  // balls that our partners have sent (which have been ignore before)
  if(theGameInfo.competitionType == COMPETITION_TYPE_MIXEDTEAM && ballsAvailableForTeamBall.size() == 0)
    findAvailableBalls(true);
  clusterBalls();
  computeModel(teamBallModel);
}

void TeamBallLocator::computeModel(TeamBallModel& teamBallModel)
{
  teamBallModel.balls.clear();
  // No active balls -> no team ball:
  if(ballsAvailableForTeamBall.size() == 0) {
    teamBallModel.isValid = false;
    return;
  }
  // Find the ball that has a) the biggest cluster and b) the highest weighting
  unsigned int bestBallIdx = 0;
  unsigned int bestBallClusterSize = unsigned(ballsAvailableForTeamBall[0].compatibleBallsIndices.size());
  float bestBallWeighting = ballsAvailableForTeamBall[0].weighting;
  for(unsigned int i=1; i<ballsAvailableForTeamBall.size(); i++)
  {
    if(ballsAvailableForTeamBall[i].compatibleBallsIndices.size() > bestBallClusterSize)
    {
      bestBallClusterSize = unsigned(ballsAvailableForTeamBall[i].compatibleBallsIndices.size());
      bestBallWeighting = ballsAvailableForTeamBall[i].weighting;
      bestBallIdx = i;
    }
    else if(ballsAvailableForTeamBall[i].compatibleBallsIndices.size() == bestBallClusterSize)
    {
      if(ballsAvailableForTeamBall[i].weighting > bestBallWeighting)
      {
        bestBallWeighting = ballsAvailableForTeamBall[i].weighting;
        bestBallIdx = i;
      }
    }
  }
  ActiveBall& bestBall = ballsAvailableForTeamBall[bestBallIdx];
  // Timestamp of model is timestamp of newest merged ball:
  teamBallModel.timeWhenLastSeen = bestBall.time;
  for(unsigned int i = 0; i<bestBall.compatibleBallsIndices.size(); i++)
  {
    ActiveBall& b = ballsAvailableForTeamBall[bestBall.compatibleBallsIndices[i]];
    if(b.time > teamBallModel.timeWhenLastSeen)
      teamBallModel.timeWhenLastSeen = b.time;
  }
  teamBallModel.timeWhenLastValid = theFrameInfo.time;
  // Count contributors to fill the team ball fields:
  int numberOfBallsByMe = 0;
  int numberOfBallsByOthers = 0;
  if(bestBall.playerNumber == static_cast<unsigned char>(theRobotInfo.number))
    numberOfBallsByMe++;
  else
    numberOfBallsByOthers++;
  teamBallModel.balls.push_back(bestBall.position);
  for(unsigned int i = 0; i<bestBall.compatibleBallsIndices.size(); i++)
  {
    ActiveBall& b = ballsAvailableForTeamBall[bestBall.compatibleBallsIndices[i]];
    teamBallModel.balls.push_back(b.position);
    if(b.playerNumber == static_cast<unsigned char>(theRobotInfo.number))
      numberOfBallsByMe++;
    else
      numberOfBallsByOthers++;
  }
  if(numberOfBallsByMe > 0) // My ball model is involved ...
  {
    if(numberOfBallsByOthers > 0)
      teamBallModel.contributors = TeamBallModel::meAndOthers;
    else
      teamBallModel.contributors = TeamBallModel::onlyMe;
  }
  else // ... or not involved
  {
    if(numberOfBallsByOthers > 1)
      teamBallModel.contributors = TeamBallModel::multipleOthers;
    else // must be == 1
      teamBallModel.contributors = TeamBallModel::oneOther;
  }
  ASSERT(numberOfBallsByOthers + numberOfBallsByMe > 0); // Just to be sure. Should never happen.
  // Finally, compute position and velocity based on the weighted clustered balls:
  float weightSum = bestBall.weighting;
  Vector2f avgPos = bestBall.position * bestBall.weighting;
  Vector2f avgVel = bestBall.velocity * bestBall.weighting;
  for(unsigned int i = 0; i<bestBall.compatibleBallsIndices.size(); i++)
  {
    ActiveBall& b = ballsAvailableForTeamBall[bestBall.compatibleBallsIndices[i]];
    weightSum += b.weighting;
    avgPos += b.position * b.weighting;
    avgVel += b.velocity * b.weighting;
  }
  if(weightSum == 0.f) // Should not happen
  {
    teamBallModel.isValid = false;
    teamBallModel.balls.clear();
  }
  else
  {
    teamBallModel.isValid = true;
    teamBallModel.position = avgPos / weightSum;
    teamBallModel.velocity = avgVel / weightSum;
  }
}

void TeamBallLocator::updateInternalBallBuffers()
{
  // I have new ball information!
  unsigned i = theRobotInfo.number;
  if(balls[i].size() == 0 ||                                          // No balls yet
     balls[i][0].time != theBallModel.timeWhenLastSeen ||             // Current ball model is newer
     balls[i][0].vel.norm() < theBallModel.estimate.velocity.norm())  // Current ball is faster. This means that we have kicked the ball but not seen it again
  {
    BufferedBall newBall;
    newBall.robotPose              = theRobotPose;
    newBall.poseValidity           = theRobotPose.validity;
    newBall.pos                    = theBallModel.estimate.position;
    newBall.vel                    = theBallModel.estimate.velocity;
    newBall.time                   = theBallModel.timeWhenLastSeen;
    newBall.seenByMixedTeamPartner = false;
    newBall.valid = true;
    balls[i].push_front(newBall);
  }
  // Observations by teammates:
  for(auto const& teammate : theTeamData.teammates)
  {
    // teammate is participating in the game and has made a new ball observation
    unsigned n = teammate.number;
    if(teammate.status == Teammate::PLAYING &&
       (balls[n].size() == 0 ||
        balls[n][0].time != teammate.theBallModel.timeWhenLastSeen ||
        balls[n][0].vel.norm() < teammate.theBallModel.estimate.velocity.norm()) &&
       teammate.theBallModel.lastPerception != Vector2f::Zero() &&
       teammate.theBallModel.estimate.position != Vector2f::Zero())
    {
      BufferedBall newBall;
      newBall.robotPose              = teammate.theRobotPose;
      newBall.poseValidity           = teammate.theRobotPose.validity;
      newBall.pos                    = teammate.theBallModel.estimate.position;
      newBall.vel                    = teammate.theBallModel.estimate.velocity;
      newBall.time                   = teammate.theBallModel.timeWhenLastSeen;
      newBall.seenByMixedTeamPartner = teammate.mateType != Teammate::BHumanRobot;
      newBall.valid = true;
      balls[teammate.number].push_front(newBall);
    }
  }
  // If any teammate is not PLAYING anymore, invalidate the observations that happened during the time
  // before the status changed:
  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.status != Teammate::PLAYING)
    {
      unsigned n = teammate.number;
      for(BufferedBall& ball : balls[n])
      {
        if(theFrameInfo.getTimeSince(ball.time) <= inactivityInvalidationTimeSpan)
          ball.valid = false;
        else
          break;
      }
    }
  }
  // If any teammate has recently changed its position significantly, all information from this robot might have been false and
  // has to be made invalid:
  for(auto const& teammate : theTeamData.teammates)
  {
    if(theFrameInfo.getTimeSince(teammate.theRobotPose.timestampLastJump) < 1000)
    {
      for(BufferedBall& ball : balls[teammate.number])
        ball.valid = false;
    }
  }
}

void TeamBallLocator::findAvailableBalls(bool considerMixedTeammates)
{
  for(unsigned int robot = 0; robot < balls.size(); ++robot)
  {
    if(balls[robot].size() > 2) // We want more than just the initial entry
    {
      // Only consider non-B-Human balls, if considerMixedTeammates is true:
      if(balls[robot][0].seenByMixedTeamPartner && !considerMixedTeammates)
        continue;
      // Find ball:
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
        const TeamBallLocator::BufferedBall& ball = balls[robot][i];
        float weighting = computeWeighting(ball);
        if(weighting != 0.f)
        {
          Vector2f absPos = ball.robotPose * ball.pos;
          Vector2f absVel = ball.vel;
          absVel.rotate(ball.robotPose.rotation);
          if(ball.time < theFrameInfo.time)
          {
            float t = (theFrameInfo.time - ball.time) / 1000.f;
            // To be technically correct, for kicked (and not seen) balls, the propagation should start at the
            // point of time of the kick and not at the point of time of the observation.
            // However, as the error is not big and as it would require to send one more timestamp,
            // this formula remains unchanged.
            BallPhysics::propagateBallPositionAndVelocity(absPos, absVel, t, theBallSpecification.friction);
          }
          // Final check to accept the ball:
          if(theFieldDimensions.isInsideCarpet(absPos))
          {
            ActiveBall newActiveBall;
            newActiveBall.position = absPos;
            newActiveBall.velocity = absVel;
            newActiveBall.playerNumber = static_cast<unsigned char>(robot);
            newActiveBall.weighting = weighting;
            newActiveBall.time = ball.time;
            ballsAvailableForTeamBall.push_back(newActiveBall);
          }
        }
      }
    }
  }
}

void TeamBallLocator::clusterBalls()
{
  for(unsigned int i=0; i<ballsAvailableForTeamBall.size(); i++)
  {
    for(unsigned int j=i+1; j<ballsAvailableForTeamBall.size(); j++)
    {
      ActiveBall& a = ballsAvailableForTeamBall[i];
      ActiveBall& b = ballsAvailableForTeamBall[j];
      if((a.position - b.position).norm() < 777.f) // There is probably a solution that is more clever ...
      {
        a.compatibleBallsIndices.push_back(j);
        b.compatibleBallsIndices.push_back(i);
      }
    }
  }
}

float TeamBallLocator::computeWeighting(const TeamBallLocator::BufferedBall& ball) const
{
  const int timeSinceBallWasSeen = theFrameInfo.getTimeSince(ball.time);
  // Hummm, the ball has not been seen for some time -> weighting is 0
  if(timeSinceBallWasSeen > ballLastSeenTimeout)
    return 0.f;

  // Computing the weighting is based on three parts:
  // 1. the time that has passed since the ball was seen the last time (smaller is better)
  // 2. the distance of the observer to the ball (closer is better)
  // 3. the validity [0,..,1] of the robot's position estimate

  // Part 1: Time *****
  // Compute a value in the interval [0,..,1], which indicates how much of the maximum possible ball age is over:
  float ballAgeRelativeToTimeout = static_cast<float>(timeSinceBallWasSeen) / static_cast<float>(ballLastSeenTimeout);
  // Use hyperbolic tangent for computing the age-based weighting.
  // The input value is multiplied by 2, this has the following effect (given the tanh curve):
  // - the ball weight is about 0.25, if half of the maximum time is over
  // - the ball weight is close to 0, if the maximum time is almost over
  float ballAgeWeight = 1.f - std::tanh(ballAgeRelativeToTimeout * 2.f);

  // Part 2: Angular distance *****
  const float camHeight = 550.f;
  const float angleOfCamera = std::atan(ball.pos.norm() / camHeight); // angle of camera when looking at the ball
  const float ballPositionWithPositiveDeviation = std::tan(angleOfCamera + 1_deg) * camHeight; // ball position when angle of camera is a bit different in position direction
  const float ballPositionWithNegativeDeviation = std::tan(angleOfCamera - 1_deg) * camHeight; // ball position when angle of camera is a bit different in negative direction
  const float ballDeviation = (ballPositionWithPositiveDeviation - ballPositionWithNegativeDeviation) / 2; // averaged deviation of the ball position

  // Part 3: Combination with validity *****
  const float weighting = (ballAgeWeight / ballDeviation) * ball.poseValidity;
  return std::abs(weighting);
}

bool TeamBallLocator::checkForResetByGameSituation()
{
  bool reset = false;
  // In READY, INITIAL, FINISHED, a team ball model does not make any sense:
  if(theGameInfo.state != STATE_PLAYING && theGameInfo.state != STATE_SET)
    reset = true;
  // Same for any goal free kick or corner kick (kick ins and pushing free kicks are different because the ball stays about where it was):
  if((theCognitionStateChanges.lastSetPlay != SET_PLAY_GOAL_FREE_KICK && theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK) ||
      (theCognitionStateChanges.lastSetPlay != SET_PLAY_CORNER_KICK && theGameInfo.setPlay == SET_PLAY_CORNER_KICK))
    reset = true;
  // If any of the above conditions was true, we delete every ball information that we have stored:
  if(reset)
  {
    for(unsigned int i = 0; i < balls.size(); ++i)
      balls[i].clear();
  }
  return reset;
}
