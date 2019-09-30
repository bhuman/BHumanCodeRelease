/**
 * @file Modules/Infrastructure/CognitionStateChangeListener.cpp
 * The file declares a module that provides CognitionStateChanges.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "CognitionStateChangeListener.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(CognitionStateChangeListener, infrastructure)

CognitionStateChangeListener::CognitionStateChangeListener() :
  lastGameState(0),
  lastGamePhase(0),
  lastPenalty(0),
  lastSetPlay(0)
{
}

void CognitionStateChangeListener::update(CognitionStateChanges& cognitionStateChanges)
{
  cognitionStateChanges.lastGameState = lastGameState;
  cognitionStateChanges.lastGamePhase = lastGamePhase;
  cognitionStateChanges.lastPenalty = lastPenalty;
  cognitionStateChanges.lastSetPlay = lastSetPlay;

  lastGameState = theGameInfo.state;
  lastGamePhase = theGameInfo.gamePhase;
  lastPenalty = theRobotInfo.penalty;
  lastSetPlay = theGameInfo.setPlay;
}
