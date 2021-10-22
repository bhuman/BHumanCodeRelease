/**
 * @file WhistleHandler.cpp
 * @author Andreas Stolpmann
 * @author Thomas Röfer
 */

#include "WhistleHandler.h"
#include "Platform/SystemCall.h"
#include "Tools/Settings.h"

MAKE_MODULE(WhistleHandler, infrastructure);

void WhistleHandler::update(GameInfo& theGameInfo)
{
  // Remember the time when the effective game state changed or we wrongly detected a change.
  if(lastGameState != theRawGameInfo.state
     || (theRawGameInfo.state == STATE_SET && checkForIllegalMotionPenalty()))
  {
    // Update timestamp, unless the raw game state is now the one already guessed.
    if(theRawGameInfo.state != guessedGameState)
      timeOfLastStateChange = theFrameInfo.time;
    guessedGameState = STATE_INITIAL;
  }
  lastGameState = theRawGameInfo.state;

  // Switching to PLAYING:
  // If the GameController sends SET, we are not guessing yet, and we hear a whistle.
  if(theRawGameInfo.state == STATE_SET && guessedGameState == STATE_INITIAL && checkForWhistle())
  {
    guessedGameState = STATE_PLAYING;
    timeOfLastStateChange = theFrameInfo.time + (theRawGameInfo.setPlay == SET_PLAY_PENALTY_KICK
                            ? ignoreWhistleAfterPenaltyKick : ignoreWhistleAfterKickOff);
    if(theRawGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT || theRawGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
      SystemCall::say("Kickoff");
  }

  // Switching to READY (not in penalty shootout):
  // If the GameController sends PLAYING, we are not guessing yet, and we hear a whistle
  // or if the GameController sends SET, we are already guessing, and hear a whistle.
  if(useWhistleAfterGoal && theRawGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT
     && ((theRawGameInfo.state == STATE_PLAYING && guessedGameState == STATE_INITIAL)
         || (theRawGameInfo.state == STATE_SET && guessedGameState == STATE_PLAYING))
     && checkForBallPosition() && checkForWhistle())
  {
    guessedGameState = STATE_READY;
    timeOfLastStateChange = theFrameInfo.time;

    // No check for any validity of the ball position, because the ball could be hidden for a while
    // before the referee finally whistles.
    kickingTeam = theBallInGoal.inOwnGoal ? theOwnTeamInfo.teamNumber : theOpponentTeamInfo.teamNumber;
    SystemCall::say((std::string("Goal for ") + TypeRegistry::getEnumName(static_cast<Settings::TeamColor>(
      kickingTeam == theOwnTeamInfo.teamNumber ? theOpponentTeamInfo.teamColor : theOwnTeamInfo.teamColor))).c_str());
  }

  // Check GameController messages for wrong switch to READY. If the GameController does not
  // send READY after a while or set plays are active, we cannot be in READY state.
  if(guessedGameState == STATE_READY &&
     (theFrameInfo.getTimeSince(timeOfLastStateChange) > gameControllerDelay + gameControllerOperatorDelay ||
      (theRawGameInfo.setPlay != SET_PLAY_NONE && theFrameInfo.getTimeSince(timeOfLastStateChange) > gameControllerOperatorDelay)))
  {
    guessedGameState = STATE_INITIAL;
    timeOfLastStateChange = std::max(timeOfLastStateChange, theFrameInfo.time - acceptPastWhistleDelay);
    SystemCall::say("Back to playing");
  }

  // Copy game state sent by GameController and override state if necessary.
  theGameInfo = theRawGameInfo;
  if(guessedGameState != STATE_INITIAL)
  {
    theGameInfo.state = guessedGameState;
    if(guessedGameState == STATE_READY)
      theGameInfo.kickingTeam = kickingTeam;
  }

  // Stop overriding when both game states match.
  if(theRawGameInfo.state == theGameInfo.state)
    guessedGameState = STATE_INITIAL;
}

bool WhistleHandler::checkForWhistle() const
{
  std::vector<const Whistle*> data;
  int numOfChannels = 0;

  if(theWhistle.channelsUsedForWhistleDetection > 0)
  {
    numOfChannels += theWhistle.channelsUsedForWhistleDetection;
    if(theWhistle.lastTimeWhistleDetected > timeOfLastStateChange)
      data.emplace_back(&theWhistle);
  }

  for(const Teammate& teammate : theTeamData.teammates)
    if(teammate.theWhistle.channelsUsedForWhistleDetection > 0)
    {
      numOfChannels += teammate.theWhistle.channelsUsedForWhistleDetection;
      if(teammate.theWhistle.lastTimeWhistleDetected > timeOfLastStateChange)
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
      if(static_cast<int>(data[j]->lastTimeWhistleDetected - data[i]->lastTimeWhistleDetected) > maxTimeDifference)
        break;

      totalConfidence += data[j]->confidenceOfLastWhistleDetection
                         * static_cast<float>(data[j]->channelsUsedForWhistleDetection);

      if(totalConfidence / numOfChannels > minAvgConfidence)
        return true;
    }
  }

  return false;
}

bool WhistleHandler::checkForIllegalMotionPenalty()
{
  penaltyTimes.resize(MAX_NUM_PLAYERS, 0);

  for(size_t i = 0; i < penaltyTimes.size(); ++i)
    if(theOwnTeamInfo.players[i].penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
      penaltyTimes[i] = 0;
    else if(penaltyTimes[i] == 0)
      penaltyTimes[i] = theFrameInfo.time;

  for(unsigned penaltyTime : penaltyTimes)
    if(penaltyTime > timeOfLastStateChange)
      return true;

  return false;
}

bool WhistleHandler::checkForBallPosition()
{
  //was in goal in the last 5 seconds
  return !checkBallForGoal || theBallInGoal.timeSinceLastInGoal <= acceptBallInGoalDelay;
}
