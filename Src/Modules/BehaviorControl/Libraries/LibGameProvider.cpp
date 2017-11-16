#include "LibGameProvider.h"

MAKE_MODULE(LibGameProvider, behaviorControl);

void LibGameProvider::update(LibGame& libGame)
{
  if(theRawGameInfo.state != STATE_PLAYING)
    libGame.didNotHearWhistleThisTime = false;

  if(theRawGameInfo.state != rawGameStateLastFrame
     && theRawGameInfo.state == STATE_PLAYING
     && theGameInfo.secondaryState != STATE2_PENALTYSHOOT)
  {
    timeWhenLastPlayingStarted = theFrameInfo.time - (lastRawSecondsRemaining - theRawGameInfo.secsRemaining);
    if((lastRawSecondsRemaining - theRawGameInfo.secsRemaining) > 10000)
      libGame.didNotHearWhistleThisTime = true;
  }
  else if(theGameInfo.state != gameStateLastFrame)
  {
    if(theGameInfo.state == STATE_READY)
      timeWhenLastReadyStarted = theFrameInfo.time;
    if(theGameInfo.state == STATE_SET)
      timeWhenLastSetStarted = theFrameInfo.time;
    if(theGameInfo.state == STATE_PLAYING)
      timeWhenLastPlayingStarted = theFrameInfo.time;
  }

  if(theBehaviorStatus.activity == BehaviorStatus::searchForBall)
    libGame.timeWhenLastSearchedBall = theFrameInfo.time;

  if(theGameInfo.dropInTime == 0 && theGameInfo.dropInTime != dropInTimeLastFrame)
    timeWhenBallWentOut = theFrameInfo.time;

  if(theCognitionStateChanges.lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE)
    timeWhenLastPenaltyEnded = theFrameInfo.time;

  libGame.timeSinceLastPenaltyEnded = theRobotInfo.penalty != PENALTY_NONE ? 0 : theFrameInfo.getTimeSince(timeWhenLastPenaltyEnded);
  libGame.lastTimeWhenBallWentOut = timeWhenBallWentOut;
  libGame.timeWhenLastReadyStarted = timeWhenLastReadyStarted;
  libGame.timeSinceReadyStarted = theGameInfo.state != STATE_READY ? 0 : theFrameInfo.getTimeSince(timeWhenLastReadyStarted);
  libGame.timeWhenLastSetStarted = timeWhenLastSetStarted;
  libGame.timeSinceSetStarted = theGameInfo.state != STATE_SET ? 0 : theFrameInfo.getTimeSince(timeWhenLastSetStarted);
  libGame.timeWhenLastPlayingStarted = timeWhenLastPlayingStarted;
  libGame.timeSincePlayingStarted = theGameInfo.state != STATE_PLAYING ? 0 : theFrameInfo.getTimeSince(timeWhenLastPlayingStarted);
  libGame.gameStateLastFrame = gameStateLastFrame;
  libGame.previousGameState = previousGameState;

  if(theGameInfo.state != gameStateLastFrame)
    previousGameState = gameStateLastFrame;
  gameStateLastFrame = theGameInfo.state;
  rawGameStateLastFrame = theRawGameInfo.state;
  dropInTimeLastFrame = theGameInfo.dropInTime;
  lastRawSecondsRemaining = theRawGameInfo.secsRemaining;
}