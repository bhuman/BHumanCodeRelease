/**
 * @file Modules/Infrastructure/CognitionStateChangeListener.h
 * The file declares a module that provides CognitionStateChanges.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Tools/Module/Module.h"

MODULE(CognitionStateChangeListener,
{,
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  PROVIDES(CognitionStateChanges),
});

/**
 * @class CognitionStateChangeLisener
 * A module that provides CognitionStateChanges.
 */
class CognitionStateChangeListener : public CognitionStateChangeListenerBase
{
private:
  int lastGameState;
  int lastGamePhase;
  int lastPenalty;
  int lastSetPlay;

public:
  CognitionStateChangeListener();
  void update(CognitionStateChanges& cognitionStateChanges) override;
};
