/**
 * @file WhistleHandler.cpp
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#include "WhistleHandler.h"

MAKE_MODULE(WhistleHandler, motionInfrastructure)

void WhistleHandler::update(GameInfo& gameInfo)
{
  gameInfo = theRawGameInfo;
  if(gameInfo.state != lastGameState && gameInfo.state == STATE_SET)
    timeOfLastSetState = theFrameInfo.time;
  lastGameState = gameInfo.state;

  if(gameInfo.state != STATE_SET)
    overrideGameState = false;
  else if(!overrideGameState && gameInfo.gameType == GAME_PLAYOFF && gameInfo.secondaryState == STATE2_NORMAL)
    overrideGameState = checkWhistle() && checkBall();

  if(overrideGameState)
  {
    if(checkForIllegalMotionPenalty())
    {
      timeOfLastSetState = theFrameInfo.time;
      overrideGameState = false;
    }
    else
    {
      gameInfo.state = STATE_PLAYING;
    }
  }
}

bool WhistleHandler::checkWhistle() const
{
  std::vector<const Whistle*> data;
  int numOfHearingRobots = 0;

  if(theWhistle.confidenceOfLastWhistleDetection != -1)
  {
    ++numOfHearingRobots;
    if(theWhistle.lastTimeWhistleDetected > timeOfLastSetState)
      data.emplace_back(&theWhistle);
  }
  for(const Teammate& teammate : theTeammateData.teammates)
  {
    if(teammate.whistle.confidenceOfLastWhistleDetection != -1)
    {
      ++numOfHearingRobots;
      if(teammate.whistle.lastTimeWhistleDetected > timeOfLastSetState)
        data.emplace_back(&teammate.whistle);
    }
  }

  std::sort(data.begin(), data.end(),
            [](const Whistle * w1, const Whistle * w2) -> bool
  {
    return w1->lastTimeWhistleDetected < w2->lastTimeWhistleDetected;
  });

  for(size_t i = 0; i < data.size(); ++i)
  {
    float totalConfidence = 0.f;
    for(size_t j = i; j < data.size(); ++j)
    {
      if(data[j]->lastTimeWhistleDetected - data[i]->lastTimeWhistleDetected > maxTimeDifference)
        break;

      totalConfidence += static_cast<float>(data[j]->confidenceOfLastWhistleDetection);
      if(totalConfidence / numOfHearingRobots > minAvgConfidence)
        return true;
    }
  }

  return false;
}

bool WhistleHandler::checkBall() const
{
  return !useBallPosition || (theTeamBallModel.isValid && theTeamBallModel.position.squaredNorm() < maxBallToMiddleDistance * maxBallToMiddleDistance);
}

bool WhistleHandler::checkForIllegalMotionPenalty()
{
  const int minPlayerNum = Global::getSettings().lowestValidPlayerNumber;
  const int maxPlayerNum = Global::getSettings().highestValidPlayerNumber;

  if(penaltyTimes.size() != static_cast<unsigned int>(maxPlayerNum))
    penaltyTimes.resize(maxPlayerNum, 0);

  for(int i = minPlayerNum; i <= maxPlayerNum; ++i)
    if(theOwnTeamInfo.players[i - 1].penalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
    {
      if(penaltyTimes[i - 1] == 0u)
        penaltyTimes[i - 1] = theFrameInfo.time;
    }
    else
    {
      penaltyTimes[i - 1] = 0u;
    }

  for(int i = minPlayerNum; i <= maxPlayerNum; ++i)
    if(penaltyTimes[i - 1] > timeOfLastSetState)
      return true;
  return false;
}
