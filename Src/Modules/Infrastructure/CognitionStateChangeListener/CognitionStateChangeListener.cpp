/**
 * @file Modules/Infrastructure/CognitionStateChangeListener.cpp
 * The file declares a module that provides CognitionStateChanges.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "CognitionStateChangeListener.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(CognitionStateChangeListener, cognitionInfrastructure)

CognitionStateChangeListener::CognitionStateChangeListener() :
  lastGameState(0),
  lastSecondaryGameState(0),
  lastPenalty(0)
{
}

void CognitionStateChangeListener::update(CognitionStateChanges& cognitionStateChanges)
{
  cognitionStateChanges.lastGameState = lastGameState;
  cognitionStateChanges.lastSecondaryGameState = lastSecondaryGameState;
  cognitionStateChanges.lastPenalty = lastPenalty;

  lastGameState = theGameInfo.state;
  lastSecondaryGameState = theGameInfo.secondaryState;
  lastPenalty = theRobotInfo.penalty;
}
