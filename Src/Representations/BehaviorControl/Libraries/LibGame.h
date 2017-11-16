/**
 * @file LibGame.h
 *
 * This file defines a representation that provide information about the course of the game states
 *
 * @author Daniel Krause
 */

#pragma once
#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibGame,
{,
  (int) timeSinceLastPenaltyEnded,
  (unsigned) lastTimeWhenBallWentOut,
  (unsigned) timeWhenLastReadyStarted,
  (unsigned)(0) timeWhenLastSearchedBall,
  (int) timeSinceReadyStarted,
  (unsigned) timeWhenLastSetStarted,
  (int) timeSinceSetStarted,
  (unsigned) timeWhenLastPlayingStarted,
  (int) timeSincePlayingStarted,
  (char) gameStateLastFrame,
  (char) previousGameState,
  (bool) didNotHearWhistleThisTime,
});
