/**
 * @file TeamBallModelProvider.cpp
 *
 * Declares a class that provides a ball model that fuses information
 * from my team and my own observation.
 *
 * @author Tim Laue
 * @contributor Liam Hurwitz
 */

#include "TeamBallModelProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Framework/Settings.h"

MAKE_MODULE(TeamBallModelProvider);

void TeamBallModelProvider::update(TeamBallModel& teamBallModel)
{
  // Check and adapt size of buffer -----
  if(balls.size() != static_cast<std::size_t>(Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1))
    balls.resize(Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1);
  // Add new observations to buffer (except for phases during which everything is deleted)
  if(!checkForResetByGameSituation())
    updateInternalBallBuffers();
  // Reset lists and variables:
  ballsAvailableForTeamBall.clear();
  teamBallModel.isValid = false;
  // Try to compute a team ball
  findAvailableBalls();
  clusterBalls();
  computeModel(teamBallModel);
  DECLARE_DEBUG_DRAWING("module:TeamBallModelProvider:bufferedBalls", "drawingOnField");
  COMPLEX_DRAWING("module:TeamBallModelProvider:bufferedBalls")
  {
    ColorRGBA bufferedBallColor(200, 200, 200, 200);
    for(unsigned int robot = 0; robot < balls.size(); ++robot)
    {
      if(balls[robot].time > 0 && balls[robot].valid)
      {
        const TeamBallModelProvider::BufferedBall& ball = balls[robot];
        Vector2f absPos = ball.robotPose * ball.pos;
        CIRCLE("module:TeamBallModelProvider:bufferedBalls", absPos.x(), absPos.y(), 150, 20, Drawings::solidPen, bufferedBallColor, Drawings::solidBrush, bufferedBallColor);
      }
    }
  }
}

void TeamBallModelProvider::computeModel(TeamBallModel& teamBallModel)
{
  // No active balls -> no team ball:
  if(ballsAvailableForTeamBall.size() == 0)
  {
    teamBallModel.isValid = false;
    return;
  }
  // Find the ball that has a) the biggest cluster and b) the highest weighting
  unsigned int bestBallIdx = 0;
  unsigned int bestBallClusterSize = unsigned(ballsAvailableForTeamBall[0].compatibleBallsIndices.size());
  float bestBallWeighting = ballsAvailableForTeamBall[0].weighting;
  for(unsigned int i = 1; i < ballsAvailableForTeamBall.size(); i++)
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
  // Finally, compute position and velocity based on the weighted clustered balls:
  float weightSum = bestBall.weighting;
  Vector2f avgPos = bestBall.position * bestBall.weighting;
  Vector2f avgVel = bestBall.velocity * bestBall.weighting;
  unsigned lastSeen = bestBall.time;
  for(unsigned int i = 0; i < bestBall.compatibleBallsIndices.size(); i++)
  {
    ActiveBall& b = ballsAvailableForTeamBall[bestBall.compatibleBallsIndices[i]];
    weightSum += b.weighting;
    avgPos += b.position * b.weighting;
    avgVel += b.velocity * b.weighting;
    lastSeen = lastSeen < b.time ? b.time : lastSeen;
  }
  teamBallModel.newerThanOwnBall = lastSeen > theWorldModelPrediction.timeWhenBallLastSeen;
  if(weightSum == 0.f) // Should not happen
  {
    teamBallModel.isValid = false;
  }
  else
  {
    teamBallModel.isValid = true;
    teamBallModel.position = avgPos / weightSum;
    teamBallModel.velocity = avgVel / weightSum;
  }
}

bool TeamBallModelProvider::isMessageRelevantForBallPosition(const BallModel& ballModel, const RobotStatus& robotStatus,
                                                             const FrameInfo& frameInfo, int ballIndex)
{
  return robotStatus.isUpright &&
         (std::abs(static_cast<int>(ballModel.timeWhenLastSeen - balls[ballIndex].time)) > timestampTolerance ||
          balls[ballIndex].vel.norm() < ballModel.estimate.velocity.norm()) &&
         frameInfo.getTimeSince(ballModel.timeWhenDisappeared) <= ballDisappearedTimeout &&
         ballModel.timeWhenLastSeen;
}

void TeamBallModelProvider::updateInternalBallBuffers()
{
  // Observations by team:
  for(auto const& message : theReceivedTeamMessages.messages)
  {
    // Validate player number
    ASSERT(message.number >= Settings::lowestValidPlayerNumber);
    ASSERT(message.number <= Settings::highestValidPlayerNumber);
    const unsigned n = message.number - Settings::lowestValidPlayerNumber;

    // If the teammate has recently changed its position significantly, all information from this robot might have been false and
    // has to be made invalid:
    if(theFrameInfo.getTimeSince(message.theRobotPose.timestampLastJump) < 1000)
    {
      balls[n].valid = false;
      continue;
    }

    // Check if the message is relevant for computing the ball position
    if(isMessageRelevantForBallPosition(message.theBallModel, message.theRobotStatus, message.theFrameInfo, n))
    {
      // Create a new buffered ball
      BufferedBall newBall;
      newBall.robotPose              = message.theRobotPose;
      newBall.poseQualityModifier    = localizationQualityToModifier(message.theRobotPose.quality);
      newBall.pos                    = message.theBallModel.estimate.position;
      newBall.vel                    = message.theBallModel.estimate.velocity;
      newBall.time                   = message.theBallModel.timeWhenLastSeen;
      newBall.valid = true;

      // Update the ball buffer if the pose quality modifier is greater than 0
      if(newBall.poseQualityModifier > 0.f) // If the value is zero, the final weighting would be 0, too.
        balls[n] = newBall;
    }
  }

  // Update the ball buffer for the last sent ball position
  const auto lastSentBall = theSentTeamMessage.theBallModel;

  // Check if the last sent ball position is different from the current one
  if(timeLastBall != lastSentBall.timeWhenLastSeen && lastSentBall.timeWhenLastSeen > 0)
  {
    // Find player id and update balls at index number with buffered Ball
    int ballIndex = theGameState.playerNumber - Settings::lowestValidPlayerNumber;

    // Check if the last sent ball position is relevant for computing the ball position
    if(theFrameInfo.getTimeSince(theSentTeamMessage.theRobotPose.timestampLastJump) < 1000)
    {
      balls[ballIndex].valid = false;
    }
    else if(isMessageRelevantForBallPosition(lastSentBall, theSentTeamMessage.theRobotStatus, theSentTeamMessage.theFrameInfo, ballIndex))
    {
      // Create a new buffered ball for the last sent ball position
      BufferedBall myBall;
      myBall.robotPose = theSentTeamMessage.theRobotPose;
      myBall.poseQualityModifier = localizationQualityToModifier(theSentTeamMessage.theRobotPose.quality);
      myBall.pos = lastSentBall.estimate.position;
      myBall.vel = lastSentBall.estimate.velocity;
      myBall.time = lastSentBall.timeWhenLastSeen;
      myBall.valid = true;
      // Update the ball buffer
      balls[ballIndex] = myBall;
      timeLastBall = lastSentBall.timeWhenLastSeen;
    }
  }

  // Invalidate ball buffers for penalized players
  for(unsigned index = 0; index < balls.size(); ++index)
  {
    const auto state = theGameState.ownTeam.playerStates[index];
    if(state == GameState::penalizedLeavingTheField ||
       state == GameState::penalizedIllegalPosition)
      balls[index].valid = false;
  }
}

void TeamBallModelProvider::findAvailableBalls()
{
  // find the own player number,
  // and then write the own BufferedBall
  for(unsigned int robot = 0; robot < balls.size(); ++robot)
  {
    if(balls[robot].valid)
    {
      const TeamBallModelProvider::BufferedBall& ball = balls[robot];
      const float weighting = computeWeighting(ball);
      if(weighting != 0.f)
      {
        Vector2f absPos = ball.robotPose * ball.pos;
        Vector2f absVel = ball.vel;
        absVel.rotate(ball.robotPose.rotation);
        if(ball.time < theFrameInfo.time)
        {
          const float t = (theFrameInfo.time - ball.time) / 1000.f;
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
          newActiveBall.playerNumber = static_cast<unsigned char>(robot + Settings::lowestValidPlayerNumber);
          newActiveBall.weighting = weighting;
          newActiveBall.time = ball.time;
          ballsAvailableForTeamBall.push_back(newActiveBall);
        }
      }
    }
  }
}

void TeamBallModelProvider::clusterBalls()
{
  for(unsigned int i = 0; i < ballsAvailableForTeamBall.size(); i++)
  {
    for(unsigned int j = i + 1; j < ballsAvailableForTeamBall.size(); j++)
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

float TeamBallModelProvider::localizationQualityToModifier(const RobotPose::LocalizationQuality quality) const
{
  if(quality == RobotPose::superb)
    return 1.f;      // If localization appears to be really good, it should not negatively affect the ball weighting
  else if(quality == RobotPose::okay)
    return 0.75f;    // This might be the most common case and the value could be a parameter (but must not)
  else
    return 0.f;      // Do not use any ball from this robot. It assumes that it does not have any clue about its position
}

float TeamBallModelProvider::computeWeighting(const TeamBallModelProvider::BufferedBall& ball) const
{
  const int timeSinceBallWasSeen = theFrameInfo.getTimeSince(ball.time);
  // The ball has not been seen for some time -> weighting is 0
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
  const float weighting = (ballAgeWeight / ballDeviation) * ball.poseQualityModifier;
  return std::abs(weighting);
}

bool TeamBallModelProvider::checkForResetByGameSituation()
{
  bool reset = false;
  // In READY, INITIAL, FINISHED, a team ball model does not make any sense:
  if(!theGameState.isPlaying() && !theGameState.isSet())
    reset = true;
  // Same for any goal kick or corner kick (kick-ins and pushing free kicks are different because the ball stays about where it was):
  if((theGameState.isGoalKick() || theGameState.isCornerKick()) &&
     theGameState.state != theExtendedGameState.stateLastFrame)
    reset = true;
  // If any of the above conditions was true, we delete every ball information that we have stored:
  if(reset)
  {
    for(unsigned int i = 0; i < balls.size(); ++i)
      balls[i].valid = false;
  }
  return reset;
}
