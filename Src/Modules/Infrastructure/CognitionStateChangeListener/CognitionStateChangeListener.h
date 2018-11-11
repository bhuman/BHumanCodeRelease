/**
 * @file Modules/Infrastructure/CgonitionStateChangeLisener.h
 * The file declares a module that provides CognitionStateChanges.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
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
