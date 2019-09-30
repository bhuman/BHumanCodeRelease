/**
 * @file WhistleHandler.cpp
 * @author Andreas Stolpmann
 */

#include "WhistleHandler.h"

MAKE_MODULE(WhistleHandler, infrastructure)

void WhistleHandler::update(GameInfo& gameInfo)
{
  if(theRawGameInfo.state == STATE_SET && theRawGameInfo.gamePhase == GAME_PHASE_NORMAL)
  {
    if(checkForIllegalMotionPenalty() || lastGameState != STATE_SET)
    {
      timeOfLastSetState = theFrameInfo.time;
      overrideGameState = false;
    }

    if(!overrideGameState)
      overrideGameState = checkWhistle() && checkBall();
  }
  else
    overrideGameState = false;

  lastGameState = theRawGameInfo.state;

  gameInfo = theRawGameInfo;
  if(overrideGameState)
    gameInfo.state = STATE_PLAYING;
}

bool WhistleHandler::checkWhistle() const
{
  std::vector<const Whistle*> data;
  int numOfChannels = 0;

  if(theWhistle.channelsUsedForWhistleDetection > 0)
  {
    numOfChannels += theWhistle.channelsUsedForWhistleDetection;
    if(theWhistle.lastTimeWhistleDetected > timeOfLastSetState)
      data.emplace_back(&theWhistle);
  }
  for(const Teammate& teammate : theTeamData.teammates)
    if(teammate.theWhistle.channelsUsedForWhistleDetection > 0)
    {
      numOfChannels += teammate.theWhistle.channelsUsedForWhistleDetection;
      if(teammate.theWhistle.lastTimeWhistleDetected > timeOfLastSetState)
        data.emplace_back(&teammate.theWhistle);
    }

  std::sort(data.begin(), data.end(),
            [](const Whistle* w1, const Whistle* w2) -> bool
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

      totalConfidence += data[j]->confidenceOfLastWhistleDetection
                         * static_cast<float>(data[j]->channelsUsedForWhistleDetection);
      if(totalConfidence / numOfChannels > minAvgConfidence)
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
  constexpr int minPlayerNum = Settings::lowestValidPlayerNumber;
  constexpr int maxPlayerNum = Settings::highestValidPlayerNumber;

  if(penaltyTimes.size() != static_cast<unsigned int>(maxPlayerNum))
    penaltyTimes.resize(maxPlayerNum, 0);

  for(int i = minPlayerNum; i <= maxPlayerNum; ++i)
  {
    if(theOwnTeamInfo.players[i - 1].penalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
    {
      if(penaltyTimes[i - 1] == 0u)
        penaltyTimes[i - 1] = theFrameInfo.time;
    }
    else
      penaltyTimes[i - 1] = 0u;
  }

  for(int i = minPlayerNum; i <= maxPlayerNum; ++i)
    if(penaltyTimes[i - 1] > timeOfLastSetState)
      return true;
  return false;
}
