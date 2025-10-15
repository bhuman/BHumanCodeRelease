/**
 * @file IndirectKickProvider.cpp
 *
 * This file implements a module that provides a representation that determines whether a Nao is allowed to shoot a goal.
 *
 * @author Ingo Kelm
 */

#include "IndirectKickProvider.h"
#include "Framework/Settings.h"

MAKE_MODULE(IndirectKickProvider);

void IndirectKickProvider::update(IndirectKick& indirectKick)
{
  // If the GameState switches from one set play to the next, indirectKick has to be reset
  // Same if the player is penalized for illegal position after a set play
  if((theGameState.state != previousGameState && theGameState.state != GameState::playing) ||
     ((theGameState.isFreeKick() || theGameControllerData.setPlay >= 1 || theGameControllerData.setPlay <= 4 ||
     theFrameInfo.time < timeSinceFreeKickEnded + refereePenaltyDelay) && playerIllegalPosition()))
    reset(indirectKick);

  // Check if more than one players is active again and set allowDirectKick to false if so
  if((lastNumOfActivePlayers <= 1 && theGameState.ownTeam.numOfActivePlayers() > 1))
    indirectKick.allowDirectKick = false;

  // Save the time when the free kick started
  if(previousGameState == GameState::playing && theGameState.isFreeKick())
    timeSinceFreeKickStarted = theFrameInfo.time;

  // Check if a kick-in for the opponent team was detected
  if(!theGameState.kickingTeamKnown && theGameState.isKickIn())
    kickInOpponentTeamDetected = checkForRefereeSignal(theGameState.leftHandTeam ? RefereeSignal::kickInLeft : RefereeSignal::kickInRight);

  // If the game state switches from a set play of the opponent team to playing, direct kicks can be allowed
  if((previousGameState == GameState::opponentGoalKick || previousGameState == GameState::opponentPushingFreeKick ||
     ((kickInOpponentTeamDetected || theGameState.kickingTeamKnown) && previousGameState == GameState::opponentKickIn)) &&
    theGameState.state == GameState::playing)
  {
    indirectKick.directKickAfterFreeKick = true;
    timeSinceFreeKickEnded = theFrameInfo.time;
    ballPositions.clear();
  } else if((theGameState.isFreeKick(previousGameState)) &&
            theGameState.state == GameState::playing)
    timeSinceFreeKickEnded = theFrameInfo.time;

  if(theGameState.isFreeKick())
    previousFreeKick = theGameState.state;

  lastNumOfActivePlayers = theGameState.ownTeam.numOfActivePlayers();
  previousGameState = theGameState.state;
  updatePlayerStates();

  // If the opponent team did not execute their free kick, a direct kick is allowed until the ball has moved or a timeout occurred
  if(indirectKick.directKickAfterFreeKick)
  {
    indirectKick.allowDirectKick = true;

    updateBallBuffer();

    if(checkBallHasMoved() || theFrameInfo.time > timeSinceFreeKickEnded + directKickTimeout ||
       theFrameInfo.time - timeSinceFreeKickStarted < freeKickDuration)
    {
      indirectKick.directKickAfterFreeKick = false;
      indirectKick.allowDirectKick = false;
      ballPositions.clear();
    }
  }

  // If the GameController is inactive (for a demo), don't use indirect kick
  // Or for Scenes with only 1 Player, even if the GameController is active (e.g. BHfast), don't use indirect kick
  if(!theGameState.gameControllerActive || theGameState.ownTeam.numOfActivePlayers() <= 1)
    indirectKick.allowDirectKick = true;
  else if(!theGameState.isForOpponentTeam())
  {
    indirectKick.lastKickTimestamp = theMotionInfo.lastKickTimestamp;

    if(!indirectKick.allowDirectKick)
    {
      updateLastBallContactTimestamps();

      for(const unsigned lastBallContactTimestamp : lastBallContactTimestamps)
      {
        // If another team member has played the ball after the last reset, direct kicks are allowed for this NAO
        if(lastBallContactTimestamp > indirectKick.lastSetPlayTime + minOwnFreeKickDelay)
        {
          indirectKick.allowDirectKick = true;
          break;
        }
      }
    }
  }
}

void IndirectKickProvider::reset(IndirectKick& indirectKick)
{
  indirectKick.allowDirectKick = false;
  indirectKick.lastSetPlayTime = theFrameInfo.time;
  indirectKick.directKickAfterFreeKick = false;
  kickInOpponentTeamDetected = false;
}

void IndirectKickProvider::updateLastBallContactTimestamps()
{
  // Gets lastKickTimestamps from other teammates, it's sufficient if every player just provides their own lastKickTimestamp (if they send a message to all teammates after kicking)
  for(const ReceivedTeamMessage& message : theReceivedTeamMessages.messages)
  {
    if(message.number >= Settings::lowestValidPlayerNumber)
      lastBallContactTimestamps[message.number - Settings::lowestValidPlayerNumber] = message.theIndirectKick.lastKickTimestamp;
  }
}

void IndirectKickProvider::updateBallBuffer()
{
  // Remove balls when the camera matrix is likely to be unstable.
  // If this should someday also work if the robot is walking, the ball buffer would need odometry updates.
  if(theMotionInfo.executedPhase != MotionPhase::stand || theIMUValueState.gyroValues.stableSinceTimestamp == theIMUValueState.timestamp)
    ballPositions.clear();
  else if(theBallModel.timeWhenLastSeen > lastBallAddedToBuffer + ballSaveInterval)
  {
    // Only add new balls every ballSaveInterval milliseconds.
    ballPositions.push_front(theBallModel.estimate.position);
    lastBallAddedToBuffer = theBallModel.timeWhenLastSeen;
  }
}

void IndirectKickProvider::updatePlayerStates()
{
  for(unsigned i = 0; i < theGameState.numOfPlayerStates; i++)
    playerStates[i] = theGameState.ownTeam.playerStates[i];
}

bool IndirectKickProvider::playerIllegalPosition()
{
  for(unsigned i = 0; i < theGameState.numOfPlayerStates; i++)
    if(theGameState.ownTeam.playerStates[i] != playerStates[i] && theGameState.ownTeam.playerStates[i] == GameState::PlayerState::penalizedIllegalPosition)
      return true;
  return false;
}

bool IndirectKickProvider::checkBallHasMoved() const
{
  // Did anyone kick the ball?
  if(theMotionInfo.lastKickTimestamp > theGameState.timeWhenStateStarted ||
     std::any_of(theTeamData.teammates.begin(), theTeamData.teammates.end(),
                 [&](const Teammate& teammate)
                 { return teammate.theIndirectKick.lastKickTimestamp > theGameState.timeWhenStateStarted; }))
    return true;

  if(theBallModel.timeWhenLastSeen > theFrameInfo.time - 500
     && (theGameState.isFreeKick(previousFreeKick) || previousFreeKick == GameState::ownPenaltyKick || previousFreeKick == GameState::opponentPenaltyKick) &&
     !theGameState.isPushingFreeKick(previousFreeKick))
  {
    const Vector2f globalBall = theRobotPose * theBallModel.estimate.position;
    //compute the covariance of the global ball model
    //start with the covariance of the local model
    Matrix2f cov = theBallModel.estimate.covariance;

    //rotate covariance matrix
    cov = Covariance::rotateCovarianceMatrix(cov, theRobotPose.rotation);

    //add covariance of the localization
    // todo: think about how to incorporate rotational uncertainty
    cov += theRobotPose.covariance.block<2, 2>(0, 0);

    //get the direction to the nearest legal position
    Vector2f direction(0.f, 0.f);
    if(theGameState.isKickIn(previousFreeKick))
      direction << 0.f, (globalBall.y() > 0 ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline) - globalBall.y();
    else if(theGameState.isGoalKick(previousFreeKick))
    {
      direction.y() = (globalBall.y() > 0 ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea) - globalBall.y();
      if(theGameState.isForOpponentTeam(previousFreeKick))
        direction.x() = theFieldDimensions.xPosOpponentGoalArea - globalBall.x();
      else
        direction.x() = theFieldDimensions.xPosOwnGoalArea - globalBall.x();
    }
    else if(theGameState.isCornerKick(previousFreeKick))
    {
      direction.y() = (globalBall.y() > 0 ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline) - globalBall.y();
      if(theGameState.isForOpponentTeam(previousFreeKick))
        direction.x() = theFieldDimensions.xPosOwnGoalLine - globalBall.x();
      else
        direction.x() = theFieldDimensions.xPosOpponentGoalLine - globalBall.x();
    }
    else if(previousFreeKick == GameState::ownPenaltyKick)
    {
      direction = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0) - globalBall;
    }
    else if(previousFreeKick == GameState::opponentPenaltyKick)
    {
      direction = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0) - globalBall;
    }

    // ball is exactly on a legal position
    if(direction.squaredNorm() == 0.f)
      return false;

    ARROW("module:IndirectKickProvider:ballCovariance", globalBall.x(), globalBall.y(),
          (globalBall + direction).x(), (globalBall + direction).y(), 10, Drawings::PenStyle::solidPen,
          ColorRGBA::red);

    // construct covariance ellipse
    float a, b, angle;
    Covariance::errorEllipse(cov, a, b, angle, confidenceIntervalToLegalPositionFree);

    // compute radius in the previous calculated direction
    Angle theta(angle);
    theta += direction.angle();
    theta.normalize();

    const float radius = (a * b) / std::sqrt(sqr(a * std::sin(theta)) + sqr(b * std::cos(theta)));

    //if the ball is further from the next legal position than the radius + a placement tolerance it is likely that the ball was already played
    if(sqr(radius + ballPlacementTolerance) < direction.squaredNorm())
      return true;
  }

  // Is there sufficient difference in recent (median filtered) ball positions?
  if(ballPositions.size() >= 6)
  {
    const auto medianOfThree = [](const Vector2f& a, const Vector2f& b, const Vector2f& c) -> const Vector2f&
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
    };
    const std::size_t firstIndex = ballPositions.size() - 1;
    const Vector2f& start = medianOfThree(ballPositions.back(), ballPositions[firstIndex - 1], ballPositions[firstIndex - 2]);
    const Vector2f& end = medianOfThree(ballPositions.front(), ballPositions[1], ballPositions[2]);
    if((start - end).squaredNorm() > sqr(ballHasMovedTolerance) * std::max(1.f, std::min(start.squaredNorm(), end.squaredNorm()) / sqr(1000.f)))
      return true;
  }

  return false;
}

bool IndirectKickProvider::checkForRefereeSignal(const RefereeSignal::Signal signal) const
{
  if(theRefereeSignal.signal == signal && theRefereeSignal.timeWhenDetected >= timeSinceFreeKickStarted)
    return true;
  else
    for(const Teammate& teammate : theTeamData.teammates)
      if(teammate.theRefereeSignal.signal == signal && teammate.theRefereeSignal.timeWhenDetected >= timeSinceFreeKickStarted)
        return true;
  return false;
}
